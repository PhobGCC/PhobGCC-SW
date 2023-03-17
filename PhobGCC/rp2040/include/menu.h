#ifndef MENU_H
#define MENU_H

enum ScreenNumber {//mark finished ones with asterisks
	MENU_SPLASH,		//0		|0
	MENU_MAIN,			//1		| 1
	MENU_STICK,			//2		|  2
	MENU_CALIBRATE,		//3		|   3
	MENU_ASTICKCAL,		//4		|    4*
	MENU_ANOTCHFIX,		//5		|    4
	MENU_CSTICKCAL,		//6		|    4*
	MENU_CNOTCHFIX,		//7		|    4
	MENU_AUTOINIT,		//8		|    4*
	MENU_STICKDBG,		//9		|    4*
	MENU_FILTER,		//10	|   3
	MENU_ASNAPBACK,		//11	|    4*
	MENU_AWAVE,			//12	|    4*
	MENU_ASMOOTH,		//13	|    4*
	MENU_CSNAPBACK,		//14	|    4*
	MENU_CWAVE,			//15	|    4*
	MENU_GEOM,			//16	|   3
	MENU_CARDINALS,		//17	|    4
	MENU_RADIUS,		//18	|    4
	MENU_SETTINGS,		//19	|  2
	MENU_SET_OVER,		//20	|   3*		Overview of all settings
	MENU_REMAP,			//21	|   3*
	MENU_RUMBLE,		//22	|   3*
	MENU_TRIGGER,		//23	|   3*		show current trigger settings and warn if illegal in melee
	MENU_LTRIGGER,		//24	|    4*
	MENU_RTRIGGER,		//25	|    4*
	MENU_TOURNEY,		//26	|	3
	MENU_RESET,			//27	|   3*
	MENU_SCOPE,			//28	|  2
	MENU_INPUTVIEW,		//29	|   3*		General
	MENU_XYSCOPE,		//30	|   3		x vs y, plotted over time
	MENU_TIMESCOPE,		//31	|   3		x/y/l/r vs time
	MENU_PRESSTIME,		//32	|   3		timing practice for wavedashing, short hopping, etc
	MENU_REACTION,		//33	|   3		reaction time test
	MENU_GAMES,			//34	|  2
	MENU_SNEK,			//35	|   3
	MENU_PING,			//36	|   3
	MENU_VISION			//37	|  2		adjust offset for phobvision
};

//The menu index consists of:
//1. Parent index
//2. Number of child indices: 255 = leaf node, 0 = one child node but not visible
//3. List of child indices (up to 6)

const uint8_t MenuIndex[38][8] = {
//   PARENT			COUNT	NEXT1			NEXT2			NEXT3			NEXT4			NEXT5			NEXT6
	{255,			0,		MENU_MAIN,		255,			255,			255,			255,			255},//splashscreen
	{MENU_SPLASH,	5,		MENU_STICK,		MENU_SETTINGS,	MENU_SCOPE,		MENU_GAMES,		MENU_VISION,	255},//main menu
	{MENU_MAIN,		3,		MENU_CALIBRATE,	MENU_FILTER,	MENU_GEOM,		255,			255,			255},//stick menu
	{MENU_STICK,	6,		MENU_ASTICKCAL,	MENU_ANOTCHFIX,	MENU_CSTICKCAL,	MENU_CNOTCHFIX,	MENU_AUTOINIT,	MENU_STICKDBG},//calibration menu
	{MENU_CALIBRATE,255,	255,			255,			255,			255,			255,			255},//astickcal
	{MENU_CALIBRATE,255,	255,			255,			255,			255,			255,			255},//anotchfix
	{MENU_CALIBRATE,255,	255,			255,			255,			255,			255,			255},//cstickcal
	{MENU_CALIBRATE,255,	255,			255,			255,			255,			255,			255},//cnotchfix
	{MENU_CALIBRATE,255,	255,			255,			255,			255,			255,			255},//autoinit
	{MENU_CALIBRATE,255,	255,			255,			255,			255,			255,			255},//stick debug info
	{MENU_STICK,	5,		MENU_ASNAPBACK,	MENU_AWAVE,		MENU_ASMOOTH,	MENU_CSNAPBACK,	MENU_CWAVE,		255},//filters
	{MENU_FILTER,	255,	255,			255,			255,			255,			255,			255},//a snapback
	{MENU_FILTER,	255,	255,			255,			255,			255,			255,			255},//a waveshaping
	{MENU_FILTER,	255,	255,			255,			255,			255,			255,			255},//a smoothing
	{MENU_FILTER,	255,	255,			255,			255,			255,			255,			255},//c snapback
	{MENU_FILTER,	255,	255,			255,			255,			255,			255,			255},//c waveshping
	{MENU_STICK,	2,		MENU_CARDINALS,	MENU_RADIUS,	255,			255,			255,			255},//geometry settings
	{MENU_GEOM,		255,	255,			255,			255,			255,			255,			255},//cardinal snapping
	{MENU_GEOM,		255,	255,			255,			255,			255,			255,			255},//radius scaling
	{MENU_MAIN,		6,		MENU_SET_OVER,	MENU_REMAP,		MENU_RUMBLE,	MENU_TRIGGER,	MENU_TOURNEY,	MENU_RESET},//settings menu
	{MENU_SETTINGS,	255,	255,			255,			255,			255,			255,			255},//setting overview
	{MENU_SETTINGS,	255,	255,			255,			255,			255,			255,			255},//remapping
	{MENU_SETTINGS,	255,	255,			255,			255,			255,			255,			255},//rumble
	{MENU_SETTINGS,	2,		MENU_LTRIGGER,	MENU_RTRIGGER,	255,			255,			255,			255},//trigger menu
	{MENU_TRIGGER,	255,	255,			255,			255,			255,			255,			255},//L trigger
	{MENU_TRIGGER,	255,	255,			255,			255,			255,			255,			255},//R trigger
	{MENU_SETTINGS,	255,	255,			255,			255,			255,			255,			255},//tourney mode (start & dup)
	{MENU_SETTINGS,	255,	255,			255,			255,			255,			255,			255},//reset
	{MENU_MAIN,		5,		MENU_INPUTVIEW,	MENU_XYSCOPE,	MENU_TIMESCOPE,	MENU_PRESSTIME,	MENU_REACTION,	255},//phobscope menu
	{MENU_SCOPE,	255,	255,			255,			255,			255,			255,			255},//input viewer
	{MENU_SCOPE,	255,	255,			255,			255,			255,			255,			255},//x/y plot
	{MENU_SCOPE,	255,	255,			255,			255,			255,			255,			255},//x/y/l/r vs time
	{MENU_SCOPE,	255,	255,			255,			255,			255,			255,			255},//button timings
	{MENU_SCOPE,	255,	255,			255,			255,			255,			255,			255},//reaction time
	{MENU_MAIN,		2,		MENU_SNEK,		MENU_PING,		255,			255,			255,			255},//games
	{MENU_GAMES,	255,	255,			255,			255,			255,			255,			255},//snek
	{MENU_GAMES,	255,	255,			255,			255,			255,			255,			255},//ping
	{MENU_MAIN,		255,	255,			255,			255,			255,			255,			255}//phobvision config
};

//The names consists of an array of null terminated c strings. Pad with spaces.
//This is the heading of the menus and also the submenu entries.

const char MenuNames[38][28] = {
	"PhobVision                 ",
	"Main Menu                  ",
	"Stick Configuration        ",
	"Stick Calibration          ",
	"Calibrate Left Stick       ",
	"Left Stick Notch Fix       ",
	"Calibrate C-Stick          ",
	"C-Stick Notch Fix          ",
	"Stick Auto-Initialize      ",
	"Stick Debug Info           ",
	"Stick Filter Settings      ",
	"Left Stick Smart Snapback  ",
	"Left Stick Waveshaping     ",
	"Left Stick Smoothing       ",
	"C-Stick Snapback           ",
	"C-Stick Waveshaping        ",
	"Stick Geometry Settings    ",
	"Cardinal Snapping          ",
	"Stick Radius Scaling       ",
	"Settings                   ",
	"Settings Overview          ",
	"Button Remapping           ",
	"Rumble Strength            ",
	"Trigger Configuration      ",
	"Left Trigger Configuration ",
	"Right Trigger Configuration",
	"Tourney Button Lockouts    ",
	"Reset Settings             ",
	"PhobScope                  ",
	"Input Viewer               ",
	"Stickmap Plots             ",
	"Value vs Time Plots        ",
	"Button Timing Viewer       ",
	"Reaction Time Test         ",
	"Games                      ",
	"Snek                       ",
	"Ping                       ",
	"PhobVision Configuration   "
};

#endif //MENU_H
