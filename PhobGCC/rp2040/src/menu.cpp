#include <cmath>
#include "cvideo.h"

#include "images/cuteGhost.h"

enum ScreenNumber {
	MENU_SPLASH,		//0		|0
	MENU_MAIN,			//1		| 1
	MENU_CALIBRATE,		//2		|  2
	MENU_ASTICKCAL,		//3		|   3
	MENU_ANOTCHADJ,		//4		|   3
	MENU_CSTICKCAL,		//5		|   3
	MENU_CNOTCHADJ,		//6		|   3
	MENU_STICKDBG,		//7		|   3
	MENU_SETTINGS,		//8		|  2
	MENU_SET_OVER,		//9		|   3		Overview of all settings
	MENU_FILTER,		//10	|   3
	MENU_ASNAPBACK,		//11	|    4
	MENU_AWAVE,			//12	|    4
	MENU_ASMOOTH,		//13	|    4
	MENU_CSNAPBACK,		//14	|    4
	MENU_CWAVE,			//15	|    4
	MENU_COFFSET,		//16	|    4
	MENU_REMAP,			//17	|   3
	MENU_RUMBLE,		//18	|   3
	MENU_TRIGGER,		//19	|   3		show current trigger settings and warn if illegal in melee
	MENU_LTRIGGER,		//20	|    4
	MENU_RTRIGGER,		//21	|    4
	MENU_RESET,			//22	|   3
	MENU_SCOPE,			//23	|  2
	MENU_INPUTVIEW,		//24	|   3		General
	MENU_XYSCOPE,		//25	|   3		x vs y, plotted over time
	MENU_TIMESCOPE,		//26	|   3		x/y/l/r vs time
	MENU_PRESSTIME,		//27	|   3		timing practice for wavedashing, short hopping, etc
	MENU_GAMES,			//28	|  2
	MENU_SNEK,			//29	|   3
	MENU_PING			//30	|   3
};

//The menu index consists of:
//1. Parent index
//2. Number of child indices: 255 = leaf node, 0 = one child node but not visible
//3. List of child indices (up to 6)

const uint8_t MenuIndex[31][8] = {
//   PARENT			COUNT	NEXT1			NEXT2			NEXT3			NEXT4			NEXT5			NEXT6
	{255,			0,		MENU_MAIN,		255,			255,			255,			255,			255},//splashscreen
	{MENU_SPLASH,	4,		MENU_CALIBRATE,	MENU_SETTINGS,	MENU_SCOPE,		MENU_GAMES,		255,			255},//main menu
	{MENU_MAIN,		5,		MENU_ASTICKCAL,	MENU_ANOTCHADJ,	MENU_CSTICKCAL,	MENU_CNOTCHADJ,	MENU_STICKDBG,	255},//calibration menu
	{MENU_CALIBRATE,255,	255,			255,			255,			255,			255,			255},//astickcal
	{MENU_CALIBRATE,255,	255,			255,			255,			255,			255,			255},//anotchadj
	{MENU_CALIBRATE,255,	255,			255,			255,			255,			255,			255},//cstickcal
	{MENU_CALIBRATE,255,	255,			255,			255,			255,			255,			255},//cnotchadj
	{MENU_CALIBRATE,255,	255,			255,			255,			255,			255,			255},//stick debug info
	{MENU_MAIN,		5,		MENU_SET_OVER,	MENU_FILTER,	MENU_REMAP,		MENU_RUMBLE,	MENU_TRIGGER,	MENU_RESET},//settings menu
	{MENU_SETTINGS,	255,	255,			255,			255,			255,			255,			255},//setting overview
	{MENU_SETTINGS,	6,		MENU_ASNAPBACK,	MENU_AWAVE,		MENU_ASMOOTH,	MENU_CSNAPBACK,	MENU_CWAVE,		MENU_COFFSET},//filters
	{MENU_FILTER,	255,	255,			255,			255,			255,			255,			255},//a snapback
	{MENU_FILTER,	255,	255,			255,			255,			255,			255,			255},//a waveshaping
	{MENU_FILTER,	255,	255,			255,			255,			255,			255,			255},//a smoothing
	{MENU_FILTER,	255,	255,			255,			255,			255,			255,			255},//c snapback
	{MENU_FILTER,	255,	255,			255,			255,			255,			255,			255},//c waveshping
	{MENU_FILTER,	255,	255,			255,			255,			255,			255,			255},//c offset
	{MENU_SETTINGS,	255,	255,			255,			255,			255,			255,			255},//remapping
	{MENU_SETTINGS,	255,	255,			255,			255,			255,			255,			255},//rumble
	{MENU_SETTINGS,	2,		MENU_LTRIGGER,	MENU_RTRIGGER,	255,			255,			255,			255},//trigger menu
	{MENU_TRIGGER,	255,	255,			255,			255,			255,			255,			255},//L trigger
	{MENU_TRIGGER,	255,	255,			255,			255,			255,			255,			255},//R trigger
	{MENU_SETTINGS,	255,	255,			255,			255,			255,			255,			255},//reset
	{MENU_MAIN,		4,		MENU_INPUTVIEW,	MENU_XYSCOPE,	MENU_TIMESCOPE,	MENU_PRESSTIME,	255,			255},//phobscope menu
	{MENU_SCOPE,	255,	255,			255,			255,			255,			255,			255},//input viewer
	{MENU_SCOPE,	255,	255,			255,			255,			255,			255,			255},//x/y plot
	{MENU_SCOPE,	255,	255,			255,			255,			255,			255,			255},//x/y/l/r vs time
	{MENU_SCOPE,	255,	255,			255,			255,			255,			255,			255},//button timings
	{MENU_MAIN,		2,		MENU_SNEK,		MENU_PING,		255,			255,			255,			255},//games
	{MENU_GAMES,	255,	255,			255,			255,			255,			255,			255},//snek
	{MENU_GAMES,	255,	255,			255,			255,			255,			255,			255}//ping
};

//The names consists of an array of null terminated c strings. Pad with spaces.
//This is the heading of the menus and also the submenu entries.

const char MenuNames[31][28] = {
	"PhobVision                 ",
	"Main Menu                  ",
	"Stick Calibration          ",
	"Calibrate Left Stick       ",
	"Left Stick Notch Adjustment",
	"Calibrate C-Stick          ",
	"C-Stick Notch Adjustment   ",
	"Stick Debug Info           ",
	"Settings                   ",
	"Settings Overview          ",
	"Stick Filter Settings      ",
	"Left Stick Smart Snapback  ",
	"Left Stick Waveshaping     ",
	"Left Stick Smoothing       ",
	"C-Stick Snapback           ",
	"C-Stick Waveshaping        ",
	"C-Stick Offset             ",
	"Button Remapping           ",
	"Rumble Strength            ",
	"Trigger Configuration      ",
	"Left Trigger Configuration ",
	"Right Trigger Configuration",
	"Reset Settings             ",
	"PhobScope                  ",
	"Input Viewer               ",
	"Stickmap Plots             ",
	"Value vs Time Plots        ",
	"Button Timing Viewer       ",
	"Games                      ",
	"Snek                       ",
	"Ping                       "
};

void drawMenu(unsigned char bitmap[],
		const unsigned int menu,
		const int itemIndex,
		const Buttons btn,
		const RawStick raw,
		const ControlConfig &controls,
		const StickParams &aStick,
		const StickParams &cStick) {
	//Basic menus
	if(MenuIndex[menu][1] == 0) {
		drawImage(bitmap, Cute_Ghost, Cute_Ghost_Index, VWIDTH/2-112, 0);//224x300
		drawString(bitmap, VWIDTH/2-105, 320, 15, "Welcome to PhobVision");
		drawString(bitmap, VWIDTH/2- 70, 340, 15, "Please press A");
	} else if (MenuIndex[menu][1] <= 6) {
		drawString(bitmap, 20, 20, 15, MenuNames[menu]);
		for(int i = 0; i < MenuIndex[menu][1]; i++) {
			drawString(bitmap, 50, 80 + 30*i, 15, MenuNames[MenuIndex[menu][i+2]]);
		}
		drawString(bitmap, 20, 80 + 30*itemIndex, 15, ">");
	} else {
		//placeholder for other screens that don't need menu graphics drawn
	}

	//big switch case to draw bottom level pages and non-text graphics
	// on other menus like in the trigger menu
	switch(menu) {
		case MENU_STICKDBG:
			drawString(bitmap,  20,  20, 15, MenuNames[menu]);
			drawString(bitmap,  30,  50, 15, "A affine");
			break;
		case MENU_SET_OVER:
			drawString(bitmap,  20,  20, 15, MenuNames[menu]);
			drawString(bitmap,  30,  50, 15, "AX SB:");
			drawString(bitmap,  30,  70, 15, "AY SB:");
			drawString(bitmap,  30,  90, 15, "AX WS:");
			drawString(bitmap,  30, 110, 15, "AY WS:");
			drawString(bitmap,  30, 130, 15, "AX SM:");
			drawString(bitmap,  30, 150, 15, "AY SM:");
			drawInt(bitmap,     90,  50, 15, 1, controls.xSnapback);
			drawInt(bitmap,     90,  70, 15, 1, controls.ySnapback);
			drawInt(bitmap,     90,  90, 15, 1, controls.axWaveshaping);
			drawInt(bitmap,     90, 110, 15, 1, controls.ayWaveshaping);
			drawInt(bitmap,     90, 130, 15, 1, controls.axSmoothing);
			drawInt(bitmap,     90, 150, 15, 1, controls.aySmoothing);
			drawString(bitmap, 150,  50, 15, "CX SB:");
			drawString(bitmap, 150,  70, 15, "CY SB:");
			drawString(bitmap, 150,  90, 15, "CX WS:");
			drawString(bitmap, 150, 110, 15, "CY WS:");
			drawString(bitmap, 150, 130, 15, "CX OF:");
			drawString(bitmap, 150, 150, 15, "CY OF:");
			drawInt(bitmap,    210,  50, 15, 1, controls.cxSmoothing);
			drawInt(bitmap,    210,  70, 15, 1, controls.cySmoothing);
			drawInt(bitmap,    210,  90, 15, 1, controls.cxWaveshaping);
			drawInt(bitmap,    210, 110, 15, 1, controls.cyWaveshaping);
			drawInt(bitmap,    210, 130, 15, 1, controls.cXOffset);
			drawInt(bitmap,    210, 150, 15, 1, controls.cYOffset);
			drawString(bitmap, 280,  50, 15, "L Mode:");
			drawString(bitmap, 280,  70, 15, "R Mode:");
			drawString(bitmap, 280,  90, 15, "L Val:");
			drawString(bitmap, 280, 110, 15, "R Val:");
			drawString(bitmap, 280, 130, 15, "L WS:");
			drawString(bitmap, 280, 150, 15, "L WS:");
			drawInt(bitmap,    350,  50, 15, 1, controls.lConfig+1);
			drawInt(bitmap,    350,  70, 15, 1, controls.rConfig+1);
			drawInt(bitmap,    350,  90, 15, 1, controls.lTriggerOffset);
			drawInt(bitmap,    350, 110, 15, 1, controls.rTriggerOffset);
			drawInt(bitmap,    350, 130, 15, 1, -1);//controls.rTriggerWaveshaping);
			drawInt(bitmap,    350, 150, 15, 1, -1);//controls.rTriggerWaveshaping);
			drawString(bitmap,  30, 170, 15, "Rumble:");
			drawInt(bitmap,    110, 170, 15, 1, controls.rumble);
			if(controls.autoInit) {
				drawString(bitmap, 30, 190, 15, "Autoinit on");
			} else {
				drawString(bitmap, 30, 190, 15, "Autoinit off");
			}
			switch(controls.jumpConfig) {
				case DEFAULTJUMP:
					drawString(bitmap, 30, 210, 15, "Normal jump");
					break;
				case SWAP_XZ:
					drawString(bitmap, 30, 210, 15, "X Z-jump");
					break;
				case SWAP_YZ:
					drawString(bitmap, 30, 210, 15, "Y Z-jump");
					break;
				case SWAP_XL:
					drawString(bitmap, 30, 210, 15, "X L-jump");
					break;
				case SWAP_XR:
					drawString(bitmap, 30, 210, 15, "X R-jump");
					break;
				case SWAP_YL:
					drawString(bitmap, 30, 210, 15, "Y L-jump");
					break;
				case SWAP_YR:
					drawString(bitmap, 30, 210, 15, "Y R-jump");
					break;
				default:
					drawString(bitmap, 30, 210, 15, "Broken jump config");
					break;
			}
			break;
		default:
			//placeholder for screens that don't have anything defined
			if(MenuIndex[menu][1] > 6) {
				drawString(bitmap, 20, 20, 15, MenuNames[menu]);
			}
	}
}

void navigateMenu(unsigned char bitmap[],
		unsigned int &menu,
		int &itemIndex,
		bool &redraw,
		Buttons &hardware,
		ControlConfig &controls) {
	static int aLockout = 0;
	if(MenuIndex[menu][1] == 0) {
		if(hardware.A) {
			aLockout = 10;
			menu = MenuIndex[menu][2];
			itemIndex = 0;
			redraw = true;
			return;
		}
	} else if(MenuIndex[menu][1] > 0) {
		static int backAccumulator = 0;
		if(hardware.B) {//back
			backAccumulator++;
		} else {
			if(backAccumulator > 0) {
				backAccumulator--;
			}
		}
		if(backAccumulator >= 30) {
			backAccumulator = 0;
			aLockout = 0;
			menu = MenuIndex[menu][0];
			itemIndex = 0;
			redraw = true;
			return;
		}

		if(MenuIndex[menu][1] <= 6) {
			//if it's a submenu, handle a, dup, and ddown
			static int duLockout = 0;
			static int ddLockout = 0;
			if(hardware.A) {
				if(aLockout == 0) {
					aLockout = 10;
					menu = MenuIndex[menu][itemIndex + 2];
					itemIndex = 0;
					redraw = true;
					return;
				}
			} else {
				//only decrement the lockout if A is released
				//it'll be unlocked after 1/6 of a second unpressed
				aLockout = fmax(0, aLockout-1);
			}
			duLockout = fmax(0, duLockout-1);
			if(hardware.Du) {
				if(duLockout == 0) {
					duLockout = 15;//a quarter of a second
					ddLockout = 0;
					itemIndex = fmax(0, itemIndex-1);
					redraw = true;
					return;
				}
			}
			ddLockout = fmax(0, ddLockout-1);
			if(hardware.Dd) {
				if(ddLockout == 0) {
					ddLockout = 15;//a quarter of a second
					duLockout = 0;
					itemIndex = fmin(MenuIndex[menu][1], itemIndex+1);
					redraw = true;
					return;
				}
			}
		} else {
			//Big switch case for controls for all the bottom level items
		}
	}
}

