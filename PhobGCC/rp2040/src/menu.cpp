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
	MENU_SETTINGS,		//7		|  2
	MENU_SET_OVER,		//8		|   3		Overview of all settings
	MENU_FILTER,		//9		|   3
	MENU_ASNAPBACK,		//10	|    4
	MENU_AWAVE,			//11	|    4
	MENU_ASMOOTH,		//12	|    4
	MENU_CSNAPBACK,		//13	|    4
	MENU_CWAVE,			//14	|    4
	MENU_COFFSET,		//15	|    4
	MENU_REMAP,			//16	|   3
	MENU_RUMBLE,		//17	|   3
	MENU_TRIGGER,		//18	|   3		show current trigger settings and warn if illegal in melee
	MENU_LTRIGGER,		//19	|    4
	MENU_RTRIGGER,		//20	|    4
	MENU_RESET,			//21	|   3
	MENU_SCOPE,			//22	|  2
	MENU_INPUTVIEW,		//23	|   3		General
	MENU_XYSCOPE,		//24	|   3		x vs y, plotted over time
	MENU_TIMESCOPE,		//25	|   3		x/y/l/r vs time
	MENU_PRESSTIME,		//26	|   3		timing practice for wavedashing, short hopping, etc
	MENU_GAMES,			//27	|  2
	MENU_SNEK,			//28	|   3
	MENU_PING			//29	|   3
};

//The menu index consists of:
//1. Parent index
//2. Number of child indices: 255 = leaf node, 0 = one child node but not visible
//3. List of child indices (up to 6)

const uint8_t MenuIndex[30][8] = {
//   PARENT			COUNT	NEXT1			NEXT2			NEXT3			NEXT4			NEXT5			NEXT6
	{255,			0,		MENU_MAIN,		255,			255,			255,			255,			255},//splashscreen
	{MENU_SPLASH,	4,		MENU_CALIBRATE,	MENU_SETTINGS,	MENU_SCOPE,		MENU_GAMES,		255,			255},//main menu
	{MENU_MAIN,		4,		MENU_ASTICKCAL,	MENU_ANOTCHADJ,	MENU_CSTICKCAL,	MENU_CNOTCHADJ,	255,			255},//calibration menu
	{MENU_CALIBRATE,255,	255,			255,			255,			255,			255,			255},//astickcal
	{MENU_CALIBRATE,255,	255,			255,			255,			255,			255,			255},//anotchadj
	{MENU_CALIBRATE,255,	255,			255,			255,			255,			255,			255},//cstickcal
	{MENU_CALIBRATE,255,	255,			255,			255,			255,			255,			255},//cnotchadj
	{MENU_MAIN,		5,		MENU_SET_OVER,	MENU_FILTER,	MENU_REMAP,		MENU_RUMBLE,	MENU_TRIGGER,	255},//settings menu
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

const char MenuNames[30][28] = {
	"PhobVision                 ",
	"Main Menu                  ",
	"Stick Calibration          ",
	"Calibrate Left Stick       ",
	"Left Stick Notch Adjustment",
	"Calibrate C-Stick          ",
	"C-Stick Notch Adjustment   ",
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
	"Reset                      ",
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
		const ControlConfig &controls) {
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
		//placeholder
		drawString(bitmap, 20, 20, 15, MenuNames[menu]);
	}

	//big switch case to draw bottom level pages and non-text graphics
	// on other menus like in the trigger menu
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

