/*-----------------------------------------------
 * Pico-Composite8      Composite Video, 8-bit output
 *
 * 2021-06-04           obstruse@earthlink.net
 *-----------------------------------------------
*/

/* Based on:
 *-----------------------------------------------
 * Title:               Pico-mposite Video Output
 *  Author:             Dean Belfield
 *  Created:            26/01/2021
 *  Last Updated:       15/02/2021
 *-----------------------------------------------
*/
/*
    Copyright (C) 2021 Bill Neisius <obstruse@earthlink.net>

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, version 3 of the License.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

/* Modified to 4 bit depth a la https://github.com/obstruse/life */

#define TESTPATTERN

#include <stdlib.h>
//#include <stdio.h>
#include "memory.h"

#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/platform.h"

#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/irq.h"

void cvideo_configure_pio_dma(PIO pio, uint sm, uint dma_channel, size_t buffer_size_words);
void cvideo_dma_handler(void);
#include "cvideo.pio.h"     // The assembled PIO code
#include "cvideo.h"
#include "cvideo_variables.h"

#include "images/cuteGhost.h"

/*-------------------------------------------------------------------*/
/*------------------Video Standard-----------------------------------*/
/*-------------------------------------------------------------------*/

const int   VIDEO_frame_lines = 525;
const int   VIDEO_frame_lines_visible = 480;
const float VIDEO_aspect_ratio = 4.0/3.0;
//const float VIDEO_horizontal_freq = 15750.0;
const float VIDEO_horizontal_freq = 15734.0;// 1000/1001 of the above
const float VIDEO_h_FP_usec = 1.5;	// front porch
const float VIDEO_h_SYNC_usec = 4.7;	// sync
const float VIDEO_h_BP_usec = 4.7;	// back porch
const float VIDEO_h_EP_usec = 2.3;	// equalizing pulse

/*-------------------------------------------------------------------*/
/*------------------Horizontal Derived-------------------------------*/
/*-------------------------------------------------------------------*/

const int   HORIZ_visible_dots = VIDEO_frame_lines_visible * VIDEO_aspect_ratio;	// full frame width
const float HORIZ_usec = 1000000.0 / VIDEO_horizontal_freq;
const float HORIZ_usec_dot = (HORIZ_usec - VIDEO_h_FP_usec - VIDEO_h_SYNC_usec - VIDEO_h_BP_usec) / HORIZ_visible_dots;
const int   HORIZ_dots = HORIZ_usec / HORIZ_usec_dot;

const int   HORIZ_bytes = HORIZ_dots / 2; // two dots per byte
const int   HORIZ_FP_bytes = VIDEO_h_FP_usec / HORIZ_usec_dot / 2;
const int   HORIZ_SYNC_bytes = VIDEO_h_SYNC_usec / HORIZ_usec_dot / 2;
const int   HORIZ_BP_bytes = VIDEO_h_BP_usec / HORIZ_usec_dot / 2;
const int   HORIZ_EP_bytes = VIDEO_h_EP_usec / HORIZ_usec_dot / 2;	// equalizing pulse during vertical sync
const int   HORIZ_pixel_start = ((HORIZ_visible_dots - VWIDTH) / 2) / 2 + HORIZ_SYNC_bytes + HORIZ_BP_bytes;

/*-------------------------------------------------------------------*/
/*------------------Vertical Derived---------------------------------*/
/*-------------------------------------------------------------------*/

const int   VERT_scanlines = VIDEO_frame_lines / 2;				// one field
const int   VERT_vblank    = (VIDEO_frame_lines - VIDEO_frame_lines_visible) / 2;	// vertical blanking, one field
const int   VERT_border    = (VERT_scanlines - VERT_vblank - VHEIGHT/2) / 2;
const int   VERT_bitmap    = VHEIGHT/2;

/*-------------------------------------------------------------------*/
/*------------------PIO----------------------------------------------*/
/*-------------------------------------------------------------------*/

const float PIO_clkdot = 1.0;        	// PIO instructions per dot
const float PIO_sysclk = 125000000.0;	// default Pico system clock
const float PIO_clkdiv = PIO_sysclk / VIDEO_horizontal_freq / PIO_clkdot / HORIZ_dots;

#define state_machine 0     // The PIO state machine to use
uint dma_channel;           // DMA channel for transferring hsync data to PIO

uint vline = 9999;          // Current video line being processed
uint bline = 0;             // Line in the bitmap to fetch
uint field = 0;		        // field, even/odd

int bmIndex = 0;
int bmCount = 0;

// bitmap buffer
unsigned char _bitmap[BUFFERLEN];

unsigned char * vsync_ll;                             // buffer for a vsync line with a long/long pulse
unsigned char * vsync_ss;                             // Buffer for an equalizing line with a short/short pulse
unsigned char * vsync_bb;                             // Buffer for a vsync blanking
unsigned char * vsync_ssb;                            // Buffer and a half for equalizing/blank line
unsigned char * border;                               // Buffer for a vsync line for the top and bottom borders
unsigned char * pixel_buffer[2];                      // Double-buffer for the pixel data scanlines

volatile bool changeBitmap = false;
volatile bool startSync = false;
volatile int frameCount = 0;

/*-------------------------------------------------------------------*/
int videoOut(const uint8_t pin_base, Buttons &btn) {

	memset(_bitmap, BLACK2, BUFFERLEN);

	vsync_ll = (unsigned char *)malloc(HORIZ_bytes);
	memset(vsync_ll, SYNC, HORIZ_bytes);				// vertical sync/serrations
	memset(vsync_ll + (HORIZ_bytes>>1) - HORIZ_EP_bytes, BLANK2, HORIZ_EP_bytes);
	memset(vsync_ll + HORIZ_bytes      - HORIZ_EP_bytes, BLANK2, HORIZ_EP_bytes);

	vsync_ss = (unsigned char *)malloc(HORIZ_bytes);
	memset(vsync_ss, BLANK2, HORIZ_bytes);				// vertical equalizing
	memset(vsync_ss, SYNC, HORIZ_EP_bytes);
	memset(vsync_ss + (HORIZ_bytes>>1), SYNC, HORIZ_EP_bytes);

	vsync_bb = (unsigned char *)malloc(HORIZ_bytes);
	memset(vsync_bb, BLANK2, HORIZ_bytes);				// vertical blanking
	memset(vsync_bb, SYNC, HORIZ_SYNC_bytes);

	vsync_ssb = (unsigned char *)malloc(HORIZ_bytes+(HORIZ_bytes>>1));
	memset(vsync_ssb, BLANK2, HORIZ_bytes + (HORIZ_bytes>>1));		// vertical equalizing/blanking
	memset(vsync_ssb, SYNC, HORIZ_EP_bytes);
	memset(vsync_ssb + (HORIZ_bytes>>1), SYNC, HORIZ_EP_bytes);

	// This bit pre-builds the border scanline and pixel buffers
	border = (unsigned char *)malloc(HORIZ_bytes);
	memset(border, GRAY2, HORIZ_bytes);			// Fill the border with the border colour
	memset(border, SYNC, HORIZ_SYNC_bytes);		        // Add the hsync pulse
	memset(border + HORIZ_SYNC_bytes,             BLANK2, HORIZ_BP_bytes);
	memset(border + HORIZ_bytes - HORIZ_FP_bytes, BLANK2, HORIZ_FP_bytes);		// front porch

	pixel_buffer[0] = (unsigned char *)malloc(HORIZ_bytes);
	memcpy(pixel_buffer[0], border, HORIZ_bytes);			// pixel buffer
	pixel_buffer[1] = (unsigned char *)malloc(HORIZ_bytes);
	memcpy(pixel_buffer[1], border, HORIZ_bytes);			// pixel buffer

	// Initialise the PIO
	PIO pio = pio0;
	uint offset = pio_add_program(pio, &cvideo_program);	// Load up the PIO program
	pio_sm_set_enabled(pio, state_machine, false);          // Disable the PIO state machine
	pio_sm_clear_fifos(pio, state_machine);	                // Clear the PIO FIFO buffers
	cvideo_initialise_pio(pio, state_machine, offset, pin_base, 4, PIO_clkdiv); // Initialise the PIO (function in cvideo.pio)

	dma_channel = dma_claim_unused_channel(true);		    // Claim a DMA channel for the hsync transfer
	cvideo_configure_pio_dma(pio, state_machine, dma_channel, HORIZ_bytes); // Hook up the DMA channel to the state machine

	// And kick everything off
	cvideo_dma_handler();
	pio_sm_set_enabled(pio, state_machine, true);           // Enable the PIO state machine

	drawImage(_bitmap, Cute_Ghost, Cute_Ghost_Index, VWIDTH/2-112, VHEIGHT/2-150);
	drawString(_bitmap, 105,  10, 13, "Hello World! +2 blah");
	drawString2x(_bitmap, 0,  50, 10, " !\"#$%&'()*+,-./0123456789");
	drawString2x(_bitmap, 1, 100, 10, " !\"#$%&'()*+,-./0123456789");
	drawString2x(_bitmap, 2, 150, 10, " !\"#$%&'()*+,-./0123456789");
	drawString2x(_bitmap, 3, 200, 10, " !\"#$%&'()*+,-./0123456789");
	drawString2x(_bitmap, 4, 250, 10, " !\"#$%&'()*+,-./0123456789");
	drawString2x(_bitmap, 5, 300, 10, " !\"#$%&'()*+,-./0123456789");
	drawString2x(_bitmap, 6, 350, 10, " !\"#$%&'()*+,-./0123456789");

	uint8_t center = 128;
	drawLine(_bitmap, center+  0, center-100, center+ 74, center- 74, WHITE);
	drawLine(_bitmap, center+100, center+  0, center+ 74, center- 74, WHITE);
	drawLine(_bitmap, center+100, center+  0, center+ 74, center+ 74, WHITE);
	drawLine(_bitmap, center+  0, center+100, center+ 74, center+ 74, WHITE);
	drawLine(_bitmap, center+  0, center+100, center- 74, center+ 74, WHITE);
	drawLine(_bitmap, center-100, center+  0, center- 74, center+ 74, WHITE);
	drawLine(_bitmap, center-100, center+  0, center- 74, center- 74, WHITE);
	drawLine(_bitmap, center+  0, center-100, center- 74, center- 74, WHITE);

	while (true) {
		//tight_loop_contents();
		while(!startSync) {
			tight_loop_contents();
		}
		startSync = false;

		memset(_bitmap, BLACK2, BUFFERLEN);

		drawLine(_bitmap, center+  0, center-100, center+ 74, center- 74, WHITE);
		drawLine(_bitmap, center+100, center+  0, center+ 74, center- 74, WHITE);
		drawLine(_bitmap, center+100, center+  0, center+ 74, center+ 74, WHITE);
		drawLine(_bitmap, center+  0, center+100, center+ 74, center+ 74, WHITE);
		drawLine(_bitmap, center+  0, center+100, center- 74, center+ 74, WHITE);
		drawLine(_bitmap, center-100, center+  0, center- 74, center+ 74, WHITE);
		drawLine(_bitmap, center-100, center+  0, center- 74, center- 74, WHITE);
		drawLine(_bitmap, center+  0, center-100, center- 74, center- 74, WHITE);
		drawString2x(_bitmap, center-8, center-16, 15, "HI");
		center++;
	}
}

/*-------------------------------------------------------------------*/
// The DMA interrupt handler
// This is triggered by DMA_IRQ_0

void __time_critical_func(cvideo_dma_handler)(void) {

	if ( ++vline <= VERT_scanlines ) {
	} else {
		vline = 0;
		bline = 0;
		field = ++field & 0x01;
		frameCount++;
	}

	if(frameCount == 1) {
		frameCount = 0;
		startSync = true;
	}

    while (true) {
		if ( vline <= VERT_vblank ) {
			switch(vline) {
				case 0:
					// for some reason interlace fails unless there's a 30usec delay here:
					busy_wait_us(HORIZ_usec/2);
					if ( !field ) {
						// odd field - blank, full line
						dma_channel_set_read_addr(dma_channel, vsync_bb, true);
			        } else {
						// even field - blank, half line
						dma_channel_set_trans_count(dma_channel, HORIZ_bytes/2, false);
						dma_channel_set_read_addr(dma_channel, vsync_bb, true);
			        }
					break;
			    case 1:
					dma_channel_set_trans_count(dma_channel, HORIZ_bytes, false);   // reset transfer size
			    case 2 ... 3:
					// send 3 vsync_ss - 'equalizing pulses'
					dma_channel_set_read_addr(dma_channel, vsync_ss, true);
					break;
				case 4 ... 6:
					// send 3 vsync_ll - 'vertical sync/serrations'
					dma_channel_set_read_addr(dma_channel, vsync_ll, true);
					break;
				case 7 ... 8:
					// send 3 vsync_ss - 'equalizing pulses'
					dma_channel_set_read_addr(dma_channel, vsync_ss, true);
					break;
				case 9:
					if ( !field ) {
						// odd field - equalizing pulse, full line
						dma_channel_set_read_addr(dma_channel, vsync_ss, true);
					} else {
						//even field - equalizing pulse, line and a half
						dma_channel_set_trans_count(dma_channel, HORIZ_bytes + HORIZ_bytes/2, false);
						dma_channel_set_read_addr(dma_channel, vsync_ssb, true);
					}
					break;
		    	case 10:
					// everything back to normal
					dma_channel_set_trans_count(dma_channel, HORIZ_bytes, false);   // reset transfer size
				default:
					// send BLANK till end of vertical blanking
					dma_channel_set_read_addr(dma_channel, vsync_bb, true);
					break;
			}
			break;
		}
		if ( vline <= VERT_vblank + VERT_border ) {
			if (changeBitmap) {
				dma_channel_set_read_addr(dma_channel, vsync_bb, true);
			} else {
				dma_channel_set_read_addr(dma_channel, border, true);
				if ( vline == VERT_vblank + VERT_border ) {
					memcpy(pixel_buffer[bline & 1] + HORIZ_pixel_start, _bitmap + (bline*2+field)*VWIDTH/2, VWIDTH/2);
				}
			}
			break;
		}
		if ( vline <= VERT_vblank + VERT_border + VERT_bitmap  ) {
			if (changeBitmap) {
				dma_channel_set_read_addr(dma_channel, vsync_bb, true);
			} else {
				dma_channel_set_read_addr(dma_channel, pixel_buffer[bline++ & 1], true);    // Set the DMA to read from one of the pixel_buffers
				memcpy(pixel_buffer[bline & 1] + HORIZ_pixel_start, _bitmap + (bline*2+field)*VWIDTH/2, VWIDTH/2);       // And memcpy the next scanline
			}
			break;
		}
		// otherwise, just output border until end of scanlines
		if (changeBitmap) {
			dma_channel_set_read_addr(dma_channel, vsync_bb, true);
		} else {
			dma_channel_set_read_addr(dma_channel, border, true);
		}
		break;
	}

	// Finally, clear the interrupt request ready for the next horizontal sync interrupt
	dma_hw->ints0 = 1u << dma_channel;
}

/*-------------------------------------------------------------------*/
// Configure the PIO DMA
// Parameters:
// - pio: The PIO to attach this to
// - sm: The state machine number
// - dma_channel: The DMA channel
// - buffer_size_words: Number of bytes to transfer
//
void cvideo_configure_pio_dma(PIO pio, uint sm, uint dma_channel, size_t buffer_size_words) {
	pio_sm_clear_fifos(pio, sm);
	dma_channel_config c = dma_channel_get_default_config(dma_channel);
	channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
	channel_config_set_read_increment(&c, true);
	channel_config_set_dreq(&c, pio_get_dreq(pio, sm, true));

	dma_channel_configure(dma_channel, &c,
	                      &pio->txf[sm],              // Destination - PIO queue
	                      vsync_bb,                   // Source - Equalizing Pulses
	                      buffer_size_words,          // Number of transfers
	                      true                        // Start - queue the Source to the Destination
	                     );

	dma_channel_set_irq0_enabled(dma_channel, true);

	irq_set_exclusive_handler(DMA_IRQ_0, cvideo_dma_handler);
	irq_set_enabled(DMA_IRQ_0, true);
}
