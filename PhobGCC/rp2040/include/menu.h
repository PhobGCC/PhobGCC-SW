#ifndef MENU_H
#define MENU_H

enum ScreenNumber {//mark finished ones with asterisks
	MENU_SPLASH,		//0		|0
	MENU_MAIN,			//1		| 1
	MENU_CALIBRATE,		//2		|  2
	MENU_ASTICKCAL,		//3		|   3
	MENU_ANOTCHFIX,		//4		|   3
	MENU_CSTICKCAL,		//5		|   3
	MENU_CNOTCHFIX,		//6		|   3
	MENU_AUTOINIT,		//7		|   3
	MENU_STICKDBG,		//8		|   3*
	MENU_SETTINGS,		//9		|  2
	MENU_SET_OVER,		//10	|   3*		Overview of all settings
	MENU_FILTER,		//11	|   3
	MENU_ASNAPBACK,		//12	|    4*
	MENU_AWAVE,			//13	|    4*
	MENU_ASMOOTH,		//14	|    4*
	MENU_CSNAPBACK,		//15	|    4*
	MENU_CWAVE,			//16	|    4*
	MENU_COFFSET,		//17	|    4*
	MENU_REMAP,			//18	|   3*
	MENU_RUMBLE,		//19	|   3*
	MENU_TRIGGER,		//20	|   3		show current trigger settings and warn if illegal in melee
	MENU_LTRIGGER,		//21	|    4
	MENU_RTRIGGER,		//22	|    4
	MENU_RESET,			//23	|   3
	MENU_SCOPE,			//24	|  2
	MENU_INPUTVIEW,		//25	|   3		General
	MENU_XYSCOPE,		//26	|   3		x vs y, plotted over time
	MENU_TIMESCOPE,		//27	|   3		x/y/l/r vs time
	MENU_PRESSTIME,		//28	|   3		timing practice for wavedashing, short hopping, etc
	MENU_GAMES,			//29	|  2
	MENU_SNEK,			//30	|   3
	MENU_PING			//31	|   3
};

//The menu index consists of:
//1. Parent index
//2. Number of child indices: 255 = leaf node, 0 = one child node but not visible
//3. List of child indices (up to 6)

const uint8_t MenuIndex[32][8] = {
//   PARENT			COUNT	NEXT1			NEXT2			NEXT3			NEXT4			NEXT5			NEXT6
	{255,			0,		MENU_MAIN,		255,			255,			255,			255,			255},//splashscreen
	{MENU_SPLASH,	4,		MENU_CALIBRATE,	MENU_SETTINGS,	MENU_SCOPE,		MENU_GAMES,		255,			255},//main menu
	{MENU_MAIN,		6,		MENU_ASTICKCAL,	MENU_ANOTCHFIX,	MENU_CSTICKCAL,	MENU_CNOTCHFIX,	MENU_AUTOINIT,	MENU_STICKDBG},//calibration menu
	{MENU_CALIBRATE,255,	255,			255,			255,			255,			255,			255},//astickcal
	{MENU_CALIBRATE,255,	255,			255,			255,			255,			255,			255},//anotchfix
	{MENU_CALIBRATE,255,	255,			255,			255,			255,			255,			255},//cstickcal
	{MENU_CALIBRATE,255,	255,			255,			255,			255,			255,			255},//cnotchfix
	{MENU_CALIBRATE,255,	255,			255,			255,			255,			255,			255},//autoinit
	{MENU_CALIBRATE,255,	255,			255,			255,			255,			255,			255},//stick debug info
	{MENU_MAIN,		6,		MENU_SET_OVER,	MENU_FILTER,	MENU_REMAP,		MENU_RUMBLE,	MENU_TRIGGER,	MENU_RESET},//settings menu
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

const char MenuNames[32][28] = {
	"PhobVision                 ",
	"Main Menu                  ",
	"Stick Calibration          ",
	"Calibrate Left Stick       ",
	"Left Stick Notch Fix       ",
	"Calibrate C-Stick          ",
	"C-Stick Notch Fix          ",
	"Stick Auto-Initialize      ",
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

#endif //MENU_H
