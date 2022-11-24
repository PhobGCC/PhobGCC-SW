#include "cvideo.h"

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
	MENU_TRIGGER,		//18	|   3
	MENU_LTRIGGER,		//19	|    4
	MENU_RTRIGGER,		//20	|    4
	MENU_SCOPE,			//21	|  2
	MENU_INPUTVIEW,		//22	|   3		General
	MENU_XYSCOPE,		//23	|   3		x vs y, plotted over time
	MENU_TIMESCOPE,		//24	|   3		x/y/l/r vs time
	MENU_PRESSTIME,		//25	|   3		timing practice for wavedashing, short hopping, etc
	MENU_GAMES,			//26	|  2
	MENU_SNEK,			//27	|   3
	MENU_PING			//28	|   3
};

//The menu index consists of:
//1. Parent index
//2. Number of child indices: 255 = leaf node, 0 = one child node but not visible
//3. List of child indices (up to 6)

const uint8_t MenuIndex[29][8] = {
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

const char MenuNames[29][28] = {
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
	"PhobScope                  ",
	"Input Viewer               ",
	"Stickmap Plots             ",
	"Value vs Time Plots        ",
	"Button Timing Viewer       ",
	"Games                      ",
	"Snek                       ",
	"Ping                       "
};

