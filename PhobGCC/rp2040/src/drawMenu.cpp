#include <cmath>
#include <cstring>
#include "pico/platform.h"
#include "cvideo.h"
#include "cvideo_variables.h"
#include "menu.h"

#include "structsAndEnums.h"
#include "menuStrings.h"
#include "images/cuteGhost.h"
#include "images/stickmaps.h"

#define ORG 127

void meleeCoordClamp(const int xIn, const int yIn, float &xOut, float &yOut) {
	const float magnitude = sqrt((float) xIn*xIn + yIn*yIn);
	const float scale = fmin(1.0f, 80.0f/magnitude);
	xOut = truncf(xIn*scale)/80.0f;
	yOut = truncf(yIn*scale)/80.0f;
}

void drawStickCal(unsigned char bitmap[],
		const unsigned int menu,
		const int itemIndex,//used for currentCalStep
		const bool changeMade, 
		const WhichStick whichStick,
		const Buttons btn,
		const RawStick raw,
		const ControlConfig &controls,
		const StickParams &aStick,
		const StickParams &cStick) {
	drawString(bitmap,  20,  20, 15, MenuNames[menu]);

	if(itemIndex < 32) {
		drawString(bitmap, 280, 20, 15, stickCal0);//measurement phase
	} else {
		drawString(bitmap, 280, 20, 15, stickCal1);//notch adjust phase
	}

	drawString(bitmap, 300,  50, 15, stickCal2);
	drawInt(bitmap,    350,  50, 15, 1, fmax(0,itemIndex));//step number
	drawString(bitmap, 380,  50, 15, "/44");

	if(itemIndex == -1 || ((itemIndex < 32) && (itemIndex % 2 == 0))) {
		drawString(bitmap, 260,  80, 15, stickCal3);
		drawString(bitmap, 260, 100, 15, stickCal4);
		if(itemIndex > 0) {
			drawString(bitmap, 260, 120, 15, stickCal5);//z to go back
		}
	} else if(itemIndex <= 15 && (itemIndex % 2 == 1)) {
		drawString(bitmap, 260,  80, 15, stickCal6);
		drawString(bitmap, 260, 100, 15, stickCal4);
		drawString(bitmap, 260, 120, 15, stickCal5);
	} else if(itemIndex <= 31 && (itemIndex % 2 == 1)) {
		drawString(bitmap, 260,  80, 15, stickCal6);
		drawString(bitmap, 260, 100, 15, stickCal4);
		drawString(bitmap, 260, 120, 15, stickCal5);
		drawString(bitmap, 260, 140, 15, stickCal7);
		drawString(bitmap, 260, 160, 15, stickCal8);
	} else {
		drawString(bitmap, 260,  80, 15, stickCal9);
		drawString(bitmap, 260, 100, 15, stickCal10);
		drawString(bitmap, 260, 120, 15, stickCal11);
		drawString(bitmap, 260, 140, 15, stickCal12);
		drawString(bitmap, 260, 160, 15, stickCal13);
		drawString(bitmap, 260, 180, 15, stickCal14);
		if(itemIndex > 32) {
			drawString(bitmap, 260, 200, 15, stickCal5);//z to go back
		}
	}

	drawString(bitmap,  30, 300, 15, stickCal15);
	drawString(bitmap,  30, 320, 15, stickCal16);
}

void drawStickCalFast(unsigned char bitmap[],
		const unsigned int menu,
		const int itemIndex,//used for currentCalStep
		const bool changeMade, 
		const WhichStick whichStick,
		const Buttons btn,
		const RawStick raw,
		const ControlConfig &controls,
		const StickParams &aStick,
		const StickParams &cStick) {
	//erase the graph
	for(int y=40; y<256+40; y++) {
		memset(bitmap + y*VWIDTHBYTE, BLACK2, 128/*256 pixels = 128 bytes*/);
	}

	int xCenter = 128;
	int yCenter = 168;//starts at 40

	//octagon
	drawLine(bitmap, xCenter+  0, yCenter-100, xCenter+74, yCenter-74, 10);
	drawLine(bitmap, xCenter+100, yCenter+  0, xCenter+74, yCenter-74, 10);
	drawLine(bitmap, xCenter+100, yCenter+  0, xCenter+74, yCenter+74, 10);
	drawLine(bitmap, xCenter+  0, yCenter+100, xCenter+74, yCenter+74, 10);
	drawLine(bitmap, xCenter+  0, yCenter+100, xCenter-74, yCenter+74, 10);
	drawLine(bitmap, xCenter-100, yCenter+  0, xCenter-74, yCenter+74, 10);
	drawLine(bitmap, xCenter-100, yCenter+  0, xCenter-74, yCenter-74, 10);
	drawLine(bitmap, xCenter+  0, yCenter-100, xCenter-74, yCenter-74, 10);

	//where to point the stick
	if(whichStick == ASTICK && itemIndex > -1) {
		drawLine(bitmap, xCenter, yCenter, xCenter+btn.Cx-ORG, yCenter-btn.Cy+ORG, 12);
	} else if(whichStick == CSTICK && itemIndex > -1) {
		drawLine(bitmap, xCenter, yCenter, xCenter+btn.Ax-ORG, yCenter-btn.Ay+ORG, 12);
	} else {
		drawLine(bitmap, xCenter, yCenter, xCenter, yCenter, 15);
	}

	//current stick position, only if currently in notch adj
	if(itemIndex >= 32 || itemIndex == -1) {
		if(whichStick == ASTICK) {
			drawLine(bitmap, xCenter+btn.Ax-ORG+1, yCenter-btn.Ay+ORG+1, xCenter+btn.Ax-ORG+1, yCenter-btn.Ay+ORG+0, 15);
			drawLine(bitmap, xCenter+btn.Ax-ORG+1, yCenter-btn.Ay+ORG-1, xCenter+btn.Ax-ORG+0, yCenter-btn.Ay+ORG-1, 15);
			drawLine(bitmap, xCenter+btn.Ax-ORG-1, yCenter-btn.Ay+ORG-1, xCenter+btn.Ax-ORG-1, yCenter-btn.Ay+ORG+0, 15);
			drawLine(bitmap, xCenter+btn.Ax-ORG-1, yCenter-btn.Ay+ORG+1, xCenter+btn.Ax-ORG+0, yCenter-btn.Ay+ORG+1, 15);
		} else {
			drawLine(bitmap, xCenter+btn.Cx-ORG+1, yCenter-btn.Cy+ORG+1, xCenter+btn.Cx-ORG+1, yCenter-btn.Cy+ORG+0, 15);
			drawLine(bitmap, xCenter+btn.Cx-ORG+1, yCenter-btn.Cy+ORG-1, xCenter+btn.Cx-ORG+0, yCenter-btn.Cy+ORG-1, 15);
			drawLine(bitmap, xCenter+btn.Cx-ORG-1, yCenter-btn.Cy+ORG-1, xCenter+btn.Cx-ORG-1, yCenter-btn.Cy+ORG+0, 15);
			drawLine(bitmap, xCenter+btn.Cx-ORG-1, yCenter-btn.Cy+ORG+1, xCenter+btn.Cx-ORG+0, yCenter-btn.Cy+ORG+1, 15);
		}
	}

	eraseCharLine(bitmap, 340);
	eraseCharLine(bitmap, 360);
	if(whichStick == ASTICK) {
		drawFloat(bitmap,   30, 340, 15, 0, 6, raw.axRaw);
		drawFloat(bitmap,   30, 360, 15, 0, 6, raw.ayRaw);
		drawFloat(bitmap,  200, 340, 15, 2, 6, raw.axUnfiltered);
		drawFloat(bitmap,  200, 360, 15, 2, 6, raw.ayUnfiltered);
		const int xCoord = btn.Ax - ORG;
		const int yCoord = btn.Ay - ORG;
		float xMelee;
		float yMelee;
		meleeCoordClamp(xCoord, yCoord, xMelee, yMelee);
		drawFloat(bitmap,  370, 340, 15, 0, 7, xMelee);
		drawFloat(bitmap,  370, 360, 15, 0, 7, yMelee);
	} else {
		drawFloat(bitmap,   30, 340, 15, 0, 6, raw.cxRaw);
		drawFloat(bitmap,   30, 360, 15, 0, 6, raw.cyRaw);
		drawFloat(bitmap,  200, 340, 15, 2, 6, raw.cxUnfiltered);
		drawFloat(bitmap,  200, 360, 15, 2, 6, raw.cyUnfiltered);
		const int xCoord = btn.Cx - ORG;
		const int yCoord = btn.Cy - ORG;
		float xMelee;
		float yMelee;
		meleeCoordClamp(xCoord, yCoord, xMelee, yMelee);
		drawFloat(bitmap,  370, 340, 15, 0, 7, xMelee);
		drawFloat(bitmap,  370, 360, 15, 0, 7, yMelee);
	}
}

void drawAutoinit(unsigned char bitmap[],
		const unsigned int menu,
		const int itemIndex,
		const bool changeMade,
		const Buttons btn,
		const RawStick raw,
		const ControlConfig &controls,
		const StickParams &aStick,
		const StickParams &cStick) {
	drawString(bitmap,  20,  20, 15, MenuNames[menu]);
	if(changeMade) {
		drawString(bitmap, 300, 20, 15, bToSave);
	}
	drawString(bitmap,  30,  50, 15, ud_only);
	drawString(bitmap,  30,  70, 15, autoinit1);
	drawString(bitmap,  30,  90, 15, autoinit2);
	drawString(bitmap,  30, 110, 15, autoinit3);
	drawString(bitmap,  30, 130, 15, autoinit4);
	if(controls.autoInit) {
		drawString(bitmap,  30, 160, 15, set_overAutoOn);
	} else {
		drawString(bitmap,  30, 160, 15, set_overAutoOff);
	}
}

void drawStickdbg(unsigned char bitmap[],
		const unsigned int menu,
		const int itemIndex,
		const bool changeMade,
		const Buttons btn,
		const RawStick raw,
		const ControlConfig &controls,
		const StickParams &aStick,
		const StickParams &cStick) {
	drawString(bitmap,  20,  20, 15, "Stick Debug Info      Press A to cycle");
	if(itemIndex == 0) {
		//raw output
		drawString(bitmap,  30,  50, 15, stickdbgARaw);
		drawString(bitmap, 280,  50, 15, stickdbgCRaw);
		drawFloat(bitmap,   30,  70, 15, 0, 6, raw.axRaw);
		drawFloat(bitmap,  280,  70, 15, 0, 6, raw.cxRaw);
		drawFloat(bitmap,   30,  90, 15, 0, 6, raw.ayRaw);
		drawFloat(bitmap,  280,  90, 15, 0, 6, raw.cyRaw);
		drawString(bitmap,  30, 120, 15, stickdbgALin);
		drawString(bitmap, 280, 120, 15, stickdbgCLin);
		drawFloat(bitmap,   30, 140, 15, 2, 6, raw.axLinearized);
		drawFloat(bitmap,  280, 140, 15, 2, 6, raw.cxLinearized);
		drawFloat(bitmap,   30, 160, 15, 2, 6, raw.ayLinearized);
		drawFloat(bitmap,  280, 160, 15, 2, 6, raw.cyLinearized);
		drawString(bitmap,  30, 190, 15, stickdbgAMap);
		drawString(bitmap, 280, 190, 15, stickdbgCMap);
		drawFloat(bitmap,   30, 210, 15, 2, 6, raw.axUnfiltered);
		drawFloat(bitmap,  280, 210, 15, 2, 6, raw.cxUnfiltered);
		drawFloat(bitmap,   30, 230, 15, 2, 6, raw.ayUnfiltered);
		drawFloat(bitmap,  280, 230, 15, 2, 6, raw.cyUnfiltered);
		drawString(bitmap,  30, 260, 15, stickdbgARnd);
		drawString(bitmap, 280, 260, 15, stickdbgCRnd);
		drawInt(bitmap,     30, 280, 15, 2, btn.Ax - ORG);
		drawInt(bitmap,    280, 280, 15, 2, btn.Cx - ORG);
		drawInt(bitmap,     30, 300, 15, 2, btn.Ay - ORG);
		drawInt(bitmap,    280, 300, 15, 2, btn.Cy - ORG);
	} else if(itemIndex == 1) {
		//fit coefficients
		drawString(bitmap,  30,  50, 15, stickdbgAXfit);
		drawFloat(bitmap,   30,  70, 15, 0, 10, aStick.fitCoeffsX[0]);
		drawFloat(bitmap,   30,  90, 15, 0, 10, aStick.fitCoeffsX[1]);
		drawFloat(bitmap,   30, 110, 15, 0, 10, aStick.fitCoeffsX[2]);
		drawFloat(bitmap,   30, 130, 15, 0, 10, aStick.fitCoeffsX[3]);
		drawString(bitmap,  30, 150, 15, stickdbgAYfit);
		drawFloat(bitmap,   30, 170, 15, 0, 10, aStick.fitCoeffsY[0]);
		drawFloat(bitmap,   30, 190, 15, 0, 10, aStick.fitCoeffsY[1]);
		drawFloat(bitmap,   30, 210, 15, 0, 10, aStick.fitCoeffsY[2]);
		drawFloat(bitmap,   30, 230, 15, 0, 10, aStick.fitCoeffsY[3]);
		drawString(bitmap, 280,  50, 15, stickdbgCXfit);
		drawFloat(bitmap,  280,  70, 15, 0, 10, cStick.fitCoeffsX[0]);
		drawFloat(bitmap,  280,  90, 15, 0, 10, cStick.fitCoeffsX[1]);
		drawFloat(bitmap,  280, 110, 15, 0, 10, cStick.fitCoeffsX[2]);
		drawFloat(bitmap,  280, 130, 15, 0, 10, cStick.fitCoeffsX[3]);
		drawString(bitmap, 280, 150, 15, stickdbgCYfit);
		drawFloat(bitmap,  280, 170, 15, 0, 10, cStick.fitCoeffsY[0]);
		drawFloat(bitmap,  280, 190, 15, 0, 10, cStick.fitCoeffsY[1]);
		drawFloat(bitmap,  280, 210, 15, 0, 10, cStick.fitCoeffsY[2]);
		drawFloat(bitmap,  280, 230, 15, 0, 10, cStick.fitCoeffsY[3]);
	} else if(itemIndex == 2) {
		//affine coefficients
		drawString(bitmap,  30,  50, 15, stickdbgAaff);
		drawString(bitmap, 280,  50, 15, stickdbgCaff);
		for(int i = 0; i < 16; i++) {
			drawFloat(bitmap,  30, 70+16*i, 15, 0, 4, aStick.affineCoeffs[i][0]);
			drawFloat(bitmap,  80, 70+16*i, 15, 0, 4, aStick.affineCoeffs[i][1]);
			drawFloat(bitmap, 130, 70+16*i, 15, 0, 4, aStick.affineCoeffs[i][2]);
			drawFloat(bitmap, 180, 70+16*i, 15, 0, 4, aStick.affineCoeffs[i][3]);
			drawFloat(bitmap, 280, 70+16*i, 15, 0, 4, cStick.affineCoeffs[i][0]);
			drawFloat(bitmap, 330, 70+16*i, 15, 0, 4, cStick.affineCoeffs[i][1]);
			drawFloat(bitmap, 380, 70+16*i, 15, 0, 4, cStick.affineCoeffs[i][2]);
			drawFloat(bitmap, 430, 70+16*i, 15, 0, 4, cStick.affineCoeffs[i][3]);
			//ends at y = 262
		}
	} else if(itemIndex == 3) {
		//boundary angles
		drawString(bitmap,  30,  50, 15, stickdbgAang);
		drawString(bitmap, 280,  50, 15, stickdbgCang);
		for(int i = 0; i < 16; i++) {
			drawFloat(bitmap,  30, 70+16*i, 15, 2, 7, aStick.boundaryAngles[i]*180/M_PI);
			drawFloat(bitmap, 280, 70+16*i, 15, 2, 7, cStick.boundaryAngles[i]*180/M_PI);
			//ends at y = 262
		}
	}
}
void drawStickdbgFast(unsigned char bitmap[],
		const unsigned int menu,
		const int itemIndex,
		const bool changeMade,
		const Buttons btn,
		const RawStick raw,
		const ControlConfig &controls,
		const StickParams &aStick,
		const StickParams &cStick) {
	if(itemIndex == 0) {
		//raw output
		eraseCharLine(bitmap, 70);
		drawFloat(bitmap,   30,  70, 15, 0, 6, raw.axRaw);
		drawFloat(bitmap,  280,  70, 15, 0, 6, raw.cxRaw);
		eraseCharLine(bitmap, 90);
		drawFloat(bitmap,   30,  90, 15, 0, 6, raw.ayRaw);
		drawFloat(bitmap,  280,  90, 15, 0, 6, raw.cyRaw);
		eraseCharLine(bitmap, 140);
		drawFloat(bitmap,   30, 140, 15, 2, 6, raw.axLinearized);
		drawFloat(bitmap,  280, 140, 15, 2, 6, raw.cxLinearized);
		eraseCharLine(bitmap, 160);
		drawFloat(bitmap,   30, 160, 15, 2, 6, raw.ayLinearized);
		drawFloat(bitmap,  280, 160, 15, 2, 6, raw.cyLinearized);
		eraseCharLine(bitmap, 210);
		drawFloat(bitmap,   30, 210, 15, 2, 6, raw.axUnfiltered);
		drawFloat(bitmap,  280, 210, 15, 2, 6, raw.cxUnfiltered);
		eraseCharLine(bitmap, 230);
		drawFloat(bitmap,   30, 230, 15, 2, 6, raw.ayUnfiltered);
		drawFloat(bitmap,  280, 230, 15, 2, 6, raw.cyUnfiltered);
		eraseCharLine(bitmap, 280);
		drawInt(bitmap,     30, 280, 15, 2, btn.Ax - ORG);
		drawInt(bitmap,    280, 280, 15, 2, btn.Cx - ORG);
		eraseCharLine(bitmap, 300);
		drawInt(bitmap,     30, 300, 15, 2, btn.Ay - ORG);
		drawInt(bitmap,    280, 300, 15, 2, btn.Cy - ORG);
	}
}

void drawAsnapback(unsigned char bitmap[],
		const unsigned int menu,
		const int itemIndex,
		const bool changeMade,
		const Buttons btn,
		const RawStick raw,
		const ControlConfig &controls,
		const StickParams &aStick,
		const StickParams &cStick) {
	drawString(bitmap,  20,  20, 15, MenuNames[menu]);
	if(changeMade) {
		drawString(bitmap, 300, 20, 15, bToSave);
	}
	drawString(bitmap,  30,  50, 15, lr_ud);
	drawString(bitmap,  30,  70, 15, asnapback1);
	drawString(bitmap,  30,  90, 15, asnapback2);
	drawString(bitmap,  30, 110, 15, asnapback3);
	drawString(bitmap,  30, 130, 15, asnapback4);
	drawString(bitmap,  30, 150, 15, asnapback5);
	drawString(bitmap,  30, 180, 15, leftStickX);
	drawInt(   bitmap, 160, 180, 15, 1, controls.xSnapback);
	drawString(bitmap, 280, 180, 15, leftStickY);
	drawInt(   bitmap, 410, 180, 15, 1, controls.ySnapback);
	if(itemIndex == 0) {
		drawString(bitmap,  10, 180, 15, arrowRight);
	} else {
		drawString(bitmap, 260, 180, 15, arrowRight);
	}
	//graph?
}

void drawAwave(unsigned char bitmap[],
		const unsigned int menu,
		const int itemIndex,
		const bool changeMade,
		const Buttons btn,
		const RawStick raw,
		const ControlConfig &controls,
		const StickParams &aStick,
		const StickParams &cStick) {
	drawString(bitmap,  20,  20, 15, MenuNames[menu]);
	if(changeMade) {
		drawString(bitmap, 300, 20, 15, bToSave);
	}
	drawString(bitmap,  30,  50, 15, lr_ud);
	drawString(bitmap,  30,  70, 15, awave1);
	drawString(bitmap,  30,  90, 15, awave2);
	drawString(bitmap,  30, 110, 15, awave3);
	drawString(bitmap,  30, 130, 15, awave4);
	drawString(bitmap,  30, 160, 15, leftStickX);
	drawInt(   bitmap, 160, 160, 15, 1, controls.axWaveshaping);
	drawString(bitmap, 280, 160, 15, leftStickY);
	drawInt(   bitmap, 410, 160, 15, 1, controls.ayWaveshaping);
	if(itemIndex == 0) {
		drawString(bitmap,  10, 160, 15, arrowRight);
	} else {
		drawString(bitmap, 260, 160, 15, arrowRight);
	}
	//graph?
}

void drawAsmooth(unsigned char bitmap[],
		const unsigned int menu,
		const int itemIndex,
		const bool changeMade,
		const Buttons btn,
		const RawStick raw,
		const ControlConfig &controls,
		const StickParams &aStick,
		const StickParams &cStick) {
	drawString(bitmap,  20,  20, 15, MenuNames[menu]);
	if(changeMade) {
		drawString(bitmap, 300, 20, 15, bToSave);
	}
	drawString(bitmap,  30,  50, 15, lr_ud);
	drawString(bitmap,  30,  70, 15, asmooth1);
	drawString(bitmap,  30,  90, 15, asmooth2);
	drawString(bitmap,  30, 110, 15, asmooth3);
	drawString(bitmap,  30, 130, 15, asmooth4);
	drawString(bitmap,  30, 160, 15, leftStickX);
	drawInt(   bitmap, 160, 160, 15, 0, controls.axSmoothing);
	drawString(bitmap, 280, 160, 15, leftStickY);
	drawInt(   bitmap, 410, 160, 15, 0, controls.aySmoothing);
	if(itemIndex == 0) {
		drawString(bitmap,  10, 160, 15, arrowRight);
	} else {
		drawString(bitmap, 260, 160, 15, arrowRight);
	}
	//graph?
}

void drawCsnapback(unsigned char bitmap[],
		const unsigned int menu,
		const int itemIndex,
		const bool changeMade,
		const Buttons btn,
		const RawStick raw,
		const ControlConfig &controls,
		const StickParams &aStick,
		const StickParams &cStick) {
			drawString(bitmap,  20,  20, 15, MenuNames[menu]);
			if(changeMade) {
				drawString(bitmap, 300, 20, 15, bToSave);
			}
			drawString(bitmap,  30,  50, 15, lr_ud);
			drawString(bitmap,  30,  70, 15, asmooth1);
			drawString(bitmap,  30,  90, 15, csmooth2);
			drawString(bitmap,  30, 110, 15, csmooth3);
			drawString(bitmap,  30, 130, 15, csmooth4);
			drawString(bitmap,  30, 160, 15, rightStickX);
			drawInt(   bitmap, 170, 160, 15, 0, controls.cxSmoothing);
			drawString(bitmap, 280, 160, 15, rightStickY);
			drawInt(   bitmap, 420, 160, 15, 0, controls.cySmoothing);
			if(itemIndex == 0) {
				drawString(bitmap,  10, 160, 15, arrowRight);
			} else {
				drawString(bitmap, 260, 160, 15, arrowRight);
			}
			//graph?
}

void drawCwave(unsigned char bitmap[],
		const unsigned int menu,
		const int itemIndex,
		const bool changeMade,
		const Buttons btn,
		const RawStick raw,
		const ControlConfig &controls,
		const StickParams &aStick,
		const StickParams &cStick) {
	drawString(bitmap,  20,  20, 15, MenuNames[menu]);
	if(changeMade) {
		drawString(bitmap, 300, 20, 15, bToSave);
	}
	drawString(bitmap,  30,  50, 15, lr_ud);
	drawString(bitmap,  30,  70, 15, awave1);
	drawString(bitmap,  30,  90, 15, awave2);
	drawString(bitmap,  30, 110, 15, cwave3);
	drawString(bitmap,  30, 130, 15, awave4);
	drawString(bitmap,  30, 160, 15, rightStickX);
	drawInt(   bitmap, 170, 160, 15, 1, controls.cxWaveshaping);
	drawString(bitmap, 280, 160, 15, rightStickY);
	drawInt(   bitmap, 420, 160, 15, 1, controls.cyWaveshaping);
	if(itemIndex == 0) {
		drawString(bitmap,  10, 160, 15, arrowRight);
	} else {
		drawString(bitmap, 260, 160, 15, arrowRight);
	}
	//graph?
}

/*
void drawCoffset(unsigned char bitmap[],
		const unsigned int menu,
		const int itemIndex,
		const bool changeMade,
		const Buttons btn,
		const RawStick raw,
		const ControlConfig &controls,
		const StickParams &aStick,
		const StickParams &cStick) {
	drawString(bitmap,  20,  20, 15, MenuNames[menu]);
	if(changeMade) {
		drawString(bitmap, 300, 20, 15, bToSave);
	}
	drawString(bitmap,  30,  50, 15, lr_ud);
	drawString(bitmap,  30,  70, 15, coffset1);
	drawString(bitmap,  30,  90, 15, coffset2);
	drawString(bitmap,  30, 110, 15, coffset3);
	drawString(bitmap,  30, 130, 15, coffset4);
	drawString(bitmap,  30, 160, 15, rightStickX);
	drawInt(   bitmap, 170, 160, 15, 2, controls.cXOffset);
	drawString(bitmap, 280, 160, 15, rightStickY);
	drawInt(   bitmap, 420, 160, 15, 2, controls.cYOffset);
	if(itemIndex == 0) {
		drawString(bitmap,  10, 160, 15, arrowRight);
	} else {
		drawString(bitmap, 260, 160, 15, arrowRight);
	}
	//graph?
}
*/

void drawCardinals(unsigned char bitmap[],
		const unsigned int menu,
		const int itemIndex,
		const bool changeMade,
		const ControlConfig &controls) {
	drawString(bitmap,  20,  20, 15, MenuNames[menu]);
	if(changeMade) {
		drawString(bitmap, 300, 20, 15, bToSave);
	}
	drawString(bitmap,  30,  50, 15, lr_ud);
	drawString(bitmap,  30,  70, 15, cardinals1);
	drawString(bitmap,  30,  90, 15, cardinals2);
	drawString(bitmap,  30, 110, 15, cardinals3);
	drawString(bitmap,  30, 130, 15, cardinals4);
	drawString(bitmap,  30, 160, 15, leftStick);
	drawInt(   bitmap, 150, 160, 15, 0, controls.astickCardinalSnapping);
	drawString(bitmap, 280, 160, 15, rightStick);
	drawInt(   bitmap, 410, 160, 15, 0, controls.cstickCardinalSnapping);
	if(itemIndex == 0) {
		drawString(bitmap,  10, 160, 15, arrowRight);
	} else {
		drawString(bitmap, 260, 160, 15, arrowRight);
	}
}

void drawRadius(unsigned char bitmap[],
		const unsigned int menu,
		const int itemIndex,
		const bool changeMade,
		const ControlConfig &controls) {
	drawString(bitmap,  20,  20, 15, MenuNames[menu]);
	if(changeMade) {
		drawString(bitmap, 300, 20, 15, bToSave);
	}
	drawString(bitmap,  30,  50, 15, lr_ud);
	drawString(bitmap,  30,  70, 15, radius1);
	drawString(bitmap,  30,  90, 15, radius2);
	drawString(bitmap,  30, 110, 15, radius3);
	drawString(bitmap,  30, 130, 15, radius4);
	drawString(bitmap,  30, 160, 15, leftStick);
	drawInt(   bitmap, 140, 160, 15, 2, controls.astickAnalogScaler);
	drawString(bitmap, 280, 160, 15, rightStick);
	drawInt(   bitmap, 400, 160, 15, 2, controls.cstickAnalogScaler);
	if(itemIndex == 0) {
		drawString(bitmap,  10, 160, 15, arrowRight);
	} else {
		drawString(bitmap, 260, 160, 15, arrowRight);
	}
}

void drawSet_over(unsigned char bitmap[],
		const unsigned int menu,
		const int itemIndex,
		const bool changeMade,
		const Buttons btn,
		const RawStick raw,
		const ControlConfig &controls,
		const StickParams &aStick,
		const StickParams &cStick) {
	drawString(bitmap,  20,  20, 15, MenuNames[menu]);
	drawString(bitmap,  30,  50, 15, "AX SB:");
	drawString(bitmap,  30,  70, 15, "AY SB:");
	drawString(bitmap,  30,  90, 15, "AX WS:");
	drawString(bitmap,  30, 110, 15, "AY WS:");
	drawString(bitmap,  30, 130, 15, "AX SM:");
	drawString(bitmap,  30, 150, 15, "AY SM:");
	drawInt(bitmap,     90,  50, 15, 2, controls.xSnapback);
	drawInt(bitmap,     90,  70, 15, 2, controls.ySnapback);
	drawInt(bitmap,     90,  90, 15, 2, controls.axWaveshaping);
	drawInt(bitmap,     90, 110, 15, 2, controls.ayWaveshaping);
	drawInt(bitmap,     90, 130, 15, 2, controls.axSmoothing);
	drawInt(bitmap,     90, 150, 15, 2, controls.aySmoothing);
	drawString(bitmap, 150,  50, 15, "CX SB:");
	drawString(bitmap, 150,  70, 15, "CY SB:");
	drawString(bitmap, 150,  90, 15, "CX WS:");
	drawString(bitmap, 150, 110, 15, "CY WS:");
	drawString(bitmap, 150, 130, 15, "A SNP:");
	drawString(bitmap, 150, 150, 15, "A SCA:");
	drawInt(bitmap,    210,  50, 15, 2, controls.cxSmoothing);
	drawInt(bitmap,    210,  70, 15, 2, controls.cySmoothing);
	drawInt(bitmap,    210,  90, 15, 2, controls.cxWaveshaping);
	drawInt(bitmap,    210, 110, 15, 2, controls.cyWaveshaping);
	drawInt(bitmap,    210, 130, 15, 2, controls.astickCardinalSnapping);
	drawInt(bitmap,    210, 150, 15, 2, controls.astickAnalogScaler);
	drawString(bitmap, 280,  50, 15, "L Mode:");
	drawString(bitmap, 280,  70, 15, "R Mode:");
	drawString(bitmap, 280,  90, 15, "L Val:");
	drawString(bitmap, 280, 110, 15, "R Val:");
	drawString(bitmap, 280, 130, 15, "C SNP:");
	drawString(bitmap, 280, 150, 15, "C SCA:");
	drawInt(bitmap,    350,  50, 15, 2, controls.lConfig+1);
	drawInt(bitmap,    350,  70, 15, 2, controls.rConfig+1);
	drawInt(bitmap,    350,  90, 15, 2, controls.lTriggerOffset);
	drawInt(bitmap,    350, 110, 15, 2, controls.rTriggerOffset);
	drawInt(bitmap,    350, 130, 15, 2, controls.cstickCardinalSnapping);
	drawInt(bitmap,    350, 150, 15, 2, controls.cstickAnalogScaler);
	drawString(bitmap,  30, 210, 15, "Rumble:");
	drawInt(bitmap,    110, 210, 15, 1, controls.rumble);
	if(controls.autoInit) {
		drawString(bitmap, 30, 230, 15, set_overAutoOn);
	} else {
		drawString(bitmap, 30, 230, 15, set_overAutoOff);
	}
	switch(controls.jumpConfig) {
		case DEFAULTJUMP:
			drawString(bitmap, 30, 250, 15, set_overJumpDf);
			break;
		case SWAP_XZ:
			drawString(bitmap, 30, 250, 15, set_overJumpXZ);
			break;
		case SWAP_YZ:
			drawString(bitmap, 30, 250, 15, set_overJumpYZ);
			break;
		case SWAP_XL:
			drawString(bitmap, 30, 250, 15, set_overJumpXL);
			break;
		case SWAP_XR:
			drawString(bitmap, 30, 250, 15, set_overJumpXR);
			break;
		case SWAP_YL:
			drawString(bitmap, 30, 250, 15, set_overJumpYL);
			break;
		case SWAP_YR:
			drawString(bitmap, 30, 250, 15, set_overJumpYR);
			break;
		default:
			drawString(bitmap, 30, 250, 15, set_overJumpBr);
			break;
	}
	switch(controls.tournamentToggle) {
		case 0:
			drawString(bitmap, 30, 270, 15, tourn0);
			break;
		case 1:
			drawString(bitmap, 30, 270, 15, tourn1);
			break;
		case 2:
			drawString(bitmap, 30, 270, 15, tourn2);
			break;
		case 3:
			drawString(bitmap, 30, 270, 15, tourn3);
			break;
		case 4:
			drawString(bitmap, 30, 270, 15, tourn4);
			break;
		case 5:
			drawString(bitmap, 30, 270, 15, tourn5);
			break;
		default:
			drawString(bitmap, 30, 270, 15, tournBr);
			break;
	}
}

void drawRemap(unsigned char bitmap[],
		const unsigned int menu,
		const int itemIndex,
		const bool changeMade,
		const Buttons btn,
		const RawStick raw,
		const ControlConfig &controls,
		const StickParams &aStick,
		const StickParams &cStick) {
	drawString(bitmap,  20,  20, 15, MenuNames[menu]);
	if(changeMade) {
		drawString(bitmap, 300, 20, 15, bToSave);
	}
	drawString(bitmap,  30,  50, 15, ud_only);
	drawString(bitmap,  30,  70, 15, remap1);
	drawString(bitmap,  30,  90, 15, remap2);
	drawString(bitmap,  30, 110, 15, remap3);
	drawString(bitmap,  30, 130, 15, remap4);
	drawString(bitmap,  30, 160, 15, currentSetting);
	switch(controls.jumpConfig) {
		case DEFAULTJUMP:
			drawString(bitmap, 200, 160, 15, remapDf);
			break;
		case SWAP_XZ:
			drawString(bitmap, 200, 160, 15, remapXZ);
			break;
		case SWAP_YZ:
			drawString(bitmap, 200, 160, 15, remapYZ);
			break;
		case SWAP_XL:
			drawString(bitmap, 200, 160, 15, remapXL);
			break;
		case SWAP_YL:
			drawString(bitmap, 200, 160, 15, remapYL);
			break;
		case SWAP_XR:
			drawString(bitmap, 200, 160, 15, remapXR);
			break;
		case SWAP_YR:
			drawString(bitmap, 200, 160, 15, remapYR);
			break;
		default:
			drawString(bitmap, 200, 160, 15, remapBr);
			break;
	}
}

void drawRumble(unsigned char bitmap[],
		const unsigned int menu,
		const int itemIndex,
		const bool changeMade,
		const Buttons btn,
		const RawStick raw,
		const ControlConfig &controls,
		const StickParams &aStick,
		const StickParams &cStick) {
	drawString(bitmap,  20,  20, 15, MenuNames[menu]);
	if(changeMade) {
		drawString(bitmap, 300, 20, 15, bToSave);
	}
	drawString(bitmap,  30,  50, 15, ud_only);
	drawString(bitmap,  30,  70, 15, rumble1);
	drawString(bitmap,  30,  90, 15, rumble2);
	drawString(bitmap,  30, 110, 15, rumble3);
	drawString(bitmap,  30, 130, 15, rumble4);
	drawString(bitmap,  30, 150, 15, rumble5);
	drawString(bitmap,  30, 180, 15, currentSetting);
	drawInt(   bitmap, 190, 180, 15, 0, controls.rumble);
}

void drawTrigger(unsigned char bitmap[],
		const unsigned int menu,
		const int itemIndex,
		const bool changeMade,
		const Buttons btn,
		const RawStick raw,
		const ControlConfig &controls,
		const StickParams &aStick,
		const StickParams &cStick) {
	//no need to draw the menu name
	drawString(bitmap,  30, 190, 15, trigger1);
	drawInt(   bitmap, 100, 190, 15, 0, controls.lConfig+1);
	drawString(bitmap, 280, 190, 15, trigger2);
	drawInt(   bitmap, 350, 190, 15, 0, controls.rConfig+1);
	drawString(bitmap,  30, 210, 15, trigger3);
	drawInt(   bitmap, 120, 210, 15, 0, controls.lTriggerOffset);
	drawString(bitmap, 280, 210, 15, trigger4);
	drawInt(   bitmap, 370, 210, 15, 0, controls.rTriggerOffset);
	if(controls.lConfig == 4) {
		if(controls.rConfig != 1 && controls.rConfig != 4 && controls.rConfig != 5) {
			drawString(bitmap, 30, 230, 15, trigger5);
		}
	} else if(controls.rConfig == 4) {
		if(controls.lConfig != 1 && controls.lConfig != 4 && controls.lConfig != 5) {
			drawString(bitmap, 30, 230, 15, trigger6);
		}
	}
	drawString(bitmap,  30, 250, 15, lrtrigger23);
	drawString(bitmap,  30, 310, 15, lrtrigger24);
}

void drawTriggerFast(unsigned char bitmap[],
		const unsigned int menu,
		const int itemIndex,
		const bool changeMade,
		const Buttons btn,
		const Buttons hardware,
		const RawStick raw,
		const ControlConfig &controls,
		const StickParams &aStick,
		const StickParams &cStick) {
	//input
	eraseCharLine(bitmap, 270);
	eraseCharLine(bitmap, 290);
	drawString(bitmap, 30, 270, 15, "LD");
	drawString(bitmap, 30, 290, 15, "RD");
	if(hardware.L) {
		drawString(bitmap, 70, 270, 15, "Pressed");
	}
	if(hardware.R) {
		drawString(bitmap, 70, 290, 15, "Pressed");
	}
	drawString(bitmap, 280, 270, 15, "LA:");
	drawInt   (bitmap, 310, 270, 15, 2, hardware.La);
	drawString(bitmap, 280, 290, 15, "RA:");
	drawInt   (bitmap, 310, 290, 15, 2, hardware.Ra);

	//output
	eraseCharLine(bitmap, 330);
	eraseCharLine(bitmap, 350);
	drawString(bitmap, 30, 330, 15, "LD");
	drawString(bitmap, 30, 350, 15, "RD");
	if(btn.L) {
		drawString(bitmap, 70, 330, 15, "Pressed");
	}
	if(btn.R) {
		drawString(bitmap, 70, 350, 15, "Pressed");
	}
	drawString(bitmap, 280, 330, 15, "LA:");
	drawInt   (bitmap, 310, 330, 15, 2, btn.La);
	drawString(bitmap, 280, 350, 15, "RA:");
	drawInt   (bitmap, 310, 350, 15, 2, btn.Ra);
}

void drawLtrigger(unsigned char bitmap[],
		const unsigned int menu,
		const int itemIndex,
		const bool changeMade,
		const Buttons btn,
		const RawStick raw,
		const ControlConfig &controls,
		const StickParams &aStick,
		const StickParams &cStick) {
	drawString(bitmap,  20,  20, 15, MenuNames[menu]);
	if(changeMade) {
		drawString(bitmap, 300, 20, 15, bToSave);
	}
	drawString(bitmap,  30,  50, 15, lr_ud);
	drawString(bitmap,  30,  70, 15, ltrigger1);
	drawString(bitmap,  30, 100, 15, lrtrigger2);
	drawInt(   bitmap,  80, 100, 15, 1, controls.lConfig+1);
	drawString(bitmap, 280, 100, 15, lrtrigger3);
	drawInt(   bitmap, 350, 100, 15, 1, controls.lTriggerOffset);
	drawString(bitmap,  30, 250, 15, lrtrigger23);
	drawString(bitmap,  30, 310, 15, lrtrigger24);
	if(itemIndex == 0) {
		drawString(bitmap,  10, 100, 15, arrowRight);
	} else {
		drawString(bitmap, 260, 100, 15, arrowRight);
	}
	switch(controls.lConfig) {
		case 0:
			drawString(bitmap,  30, 130, 15, lrtrigger4);
			if(controls.rConfig == 4) {drawString(bitmap, 30, 150, 15, l5conflict);}
			break;
		case 1:
			drawString(bitmap,  30, 130, 15, lrtrigger5);
			drawString(bitmap,  30, 150, 15, lrtrigger6);
			break;
		case 2:
			drawString(bitmap,  30, 130, 15, lrtrigger7);
			drawString(bitmap,  30, 150, 15, lrtrigger8);
			if(controls.rConfig == 4) {drawString(bitmap, 30, 170, 15, l5conflict);}
			break;
		case 3:
			drawString(bitmap,  30, 130, 15, lrtrigger9);
			drawInt(   bitmap, 400, 130, 15, 0, controls.lTriggerOffset);
			drawString(bitmap,  30, 150, 15, lrtrigger10);
			drawString(bitmap,  30, 170, 15, lrtrigger11);
			if(controls.lTriggerOffset < 80) {drawString(bitmap, 30, 190, 15, lrultimate);}
			if(controls.rConfig == 4) {drawString(bitmap, 30, 210, 15, l5conflict);}
			break;
		case 4:
			drawString(bitmap,  30, 130, 15, lrtrigger12);
			drawString(bitmap,  30, 150, 15, lrtrigger8);//reused
			drawString(bitmap,  30, 170, 15, lrtrigger13);
			drawString(bitmap,  30, 190, 15, lrtrigger14);
			if(controls.lTriggerOffset < 80) {drawString(bitmap, 30, 210, 15, lrultimate);}
			if(controls.rConfig != 1 && controls.rConfig != 4 && controls.rConfig != 5) {
				drawString(bitmap, 30, 230, 15, trigger5);
			}
			break;
		case 5:
			drawString(bitmap,  30, 130, 15, lrtrigger15);
			drawString(bitmap,  30, 150, 15, lrtrigger16);
			drawString(bitmap,  30, 170, 15, lrtrigger17);
			drawString(bitmap,  30, 190, 15, lrtrigger18);
			if(controls.lTriggerOffset < 80) {drawString(bitmap, 30, 210, 15, lrultimate);}
			break;
		case 6:
			drawString(bitmap,  30, 130, 15, lrtrigger19);
			drawString(bitmap,  30, 150, 15, lrtrigger20);
			drawString(bitmap,  30, 170, 15, lrtrigger21);
			drawString(bitmap,  30, 190, 15, lrtrigger22);
			drawFloat( bitmap, 140, 190, 15, 0, 6, (0.0112f * controls.lTriggerOffset) + 0.4494f);
			if(controls.rConfig == 4) {drawString(bitmap, 30, 210, 15, l5conflict);}
			break;
	}
	//graph?
}

void drawRtrigger(unsigned char bitmap[],
		const unsigned int menu,
		const int itemIndex,
		const bool changeMade,
		const Buttons btn,
		const RawStick raw,
		const ControlConfig &controls,
		const StickParams &aStick,
		const StickParams &cStick) {
	drawString(bitmap,  20,  20, 15, MenuNames[menu]);
	if(changeMade) {
		drawString(bitmap, 300, 20, 15, bToSave);
	}
	drawString(bitmap,  30,  50, 15, lr_ud);
	drawString(bitmap,  30,  70, 15, rtrigger1);
	drawString(bitmap,  30, 100, 15, lrtrigger2);
	drawInt(   bitmap,  80, 100, 15, 1, controls.rConfig+1);
	drawString(bitmap, 280, 100, 15, lrtrigger3);
	drawInt(   bitmap, 350, 100, 15, 1, controls.rTriggerOffset);
	drawString(bitmap,  30, 250, 15, lrtrigger23);
	drawString(bitmap,  30, 310, 15, lrtrigger24);
	if(itemIndex == 0) {
		drawString(bitmap,  10, 100, 15, arrowRight);
	} else {
		drawString(bitmap, 260, 100, 15, arrowRight);
	}
	switch(controls.rConfig) {
		case 0:
			drawString(bitmap,  30, 130, 15, lrtrigger4);
			if(controls.lConfig == 4) {drawString(bitmap, 30, 150, 15, r5conflict);}
			break;
		case 1:
			drawString(bitmap,  30, 130, 15, lrtrigger5);
			drawString(bitmap,  30, 150, 15, lrtrigger6);
			break;
		case 2:
			drawString(bitmap,  30, 130, 15, lrtrigger7);
			drawString(bitmap,  30, 150, 15, lrtrigger8);
			if(controls.lConfig == 4) {drawString(bitmap, 30, 170, 15, r5conflict);}
			break;
		case 3:
			drawString(bitmap,  30, 130, 15, lrtrigger9);
			drawInt(   bitmap, 400, 130, 15, 0, controls.rTriggerOffset);
			drawString(bitmap,  30, 150, 15, lrtrigger10);
			drawString(bitmap,  30, 170, 15, lrtrigger11);
			if(controls.rTriggerOffset < 80) {drawString(bitmap, 30, 190, 15, lrultimate);}
			if(controls.lConfig == 4) {drawString(bitmap, 30, 210, 15, r5conflict);}
			break;
		case 4:
			drawString(bitmap,  30, 130, 15, lrtrigger12);
			drawString(bitmap,  30, 150, 15, lrtrigger8);//reused
			drawString(bitmap,  30, 170, 15, lrtrigger13);
			drawString(bitmap,  30, 190, 15, lrtrigger14);
			if(controls.rTriggerOffset < 80) {drawString(bitmap, 30, 210, 15, lrultimate);}
			if(controls.lConfig != 1 && controls.lConfig != 4 && controls.lConfig != 5) {
				drawString(bitmap, 30, 230, 15, trigger6);
			}
			break;
		case 5:
			drawString(bitmap,  30, 130, 15, lrtrigger15);
			drawString(bitmap,  30, 150, 15, lrtrigger16);
			drawString(bitmap,  30, 170, 15, lrtrigger17);
			drawString(bitmap,  30, 190, 15, lrtrigger18);
			if(controls.rTriggerOffset < 80) {drawString(bitmap, 30, 210, 15, lrultimate);}
			break;
		case 6:
			drawString(bitmap,  30, 130, 15, lrtrigger19);
			drawString(bitmap,  30, 150, 15, lrtrigger20);
			drawString(bitmap,  30, 170, 15, lrtrigger21);
			drawString(bitmap,  30, 190, 15, lrtrigger22);
			drawFloat( bitmap, 140, 190, 15, 0, 6, (0.0112f * controls.rTriggerOffset) + 0.4494f);
			if(controls.lConfig == 4) {drawString(bitmap, 30, 210, 15, r5conflict);}
			break;
	}
	//graph?
}

void drawTourney(unsigned char bitmap[],
		const unsigned int menu,
		const bool changeMade,
		const ControlConfig &controls) {
	drawString(bitmap,  20,  20, 15, MenuNames[menu]);
	if(changeMade) {
		drawString(bitmap, 300, 20, 15, bToSave);
	}
	drawString(bitmap,  30,  50, 15, ud_only);
	drawString(bitmap,  30,  70, 15, tourney1);
	drawString(bitmap,  30,  90, 15, tourney2);
	drawString(bitmap,  30, 110, 15, tourney3);
	drawString(bitmap,  30, 130, 15, tourney4);
	drawString(bitmap,  30, 160, 15, currentSetting);
	switch(controls.tournamentToggle) {
		case 0:
			drawString(bitmap, 200, 160, 15, tourn0);
			break;
		case 1:
			drawString(bitmap, 200, 160, 15, tourn1);
			break;
		case 2:
			drawString(bitmap, 200, 160, 15, tourn2);
			break;
		case 3:
			drawString(bitmap, 200, 160, 15, tourn3);
			break;
		case 4:
			drawString(bitmap, 200, 160, 15, tourn4);
			break;
		case 5:
			drawString(bitmap, 200, 160, 15, tourn5);
			break;
		default:
			drawString(bitmap, 200, 160, 15, tournBr);
			break;
	}
}

void drawReset(unsigned char bitmap[],
		const unsigned int menu,
		const int itemIndex,
		const bool changeMade,
		const Buttons btn,
		const RawStick raw,
		const ControlConfig &controls,
		const StickParams &aStick,
		const StickParams &cStick) {
	drawString(bitmap,  20,  20, 15, MenuNames[menu]);
	drawString(bitmap,  30,  50, 15, reset1);
	drawString(bitmap,  30, 120, 15, reset7);
	drawString(bitmap,  30, 150, 15, reset8);
	if(itemIndex == 0 || itemIndex == 2) {//soft reset
		drawString(bitmap,  30,  70, 15, reset2);
		drawString(bitmap,  30,  90, 15, reset3);
		drawString(bitmap,  10, 120, 15, arrowRight);
	} else {
		drawString(bitmap,  30,  70, 15, reset4);
		drawString(bitmap,  30,  90, 15, reset5);
		drawString(bitmap,  10, 150, 15, arrowRight);
	}
	if(itemIndex >= 2) {//confirm
		drawString(bitmap,  30, 180, 15, reset6);
	}
}

void drawInputview(unsigned char bitmap[],
		const unsigned int menu,
		const int itemIndex,
		const bool changeMade,
		const Buttons btn,
		const RawStick raw,
		const ControlConfig &controls,
		const StickParams &aStick,
		const StickParams &cStick) {
	drawString(bitmap,  20,  20, 15, MenuNames[menu]);
	drawString(bitmap, 280,  50, 15, inputview1);
	drawString(bitmap, 280, 160, 15, inputview2);
	drawString(bitmap,  30, 300, 15, inputview3);
	drawString(bitmap,  30, 320, 15, inputview4);
}

void drawInputviewFast(unsigned char bitmap[],
		const unsigned int menu,
		const int itemIndex,
		const bool changeMade,
		const Buttons btn,
		const Buttons hardware,
		const RawStick raw,
		const ControlConfig &controls,
		const StickParams &aStick,
		const StickParams &cStick) {
	//Button Inputs
	eraseCharLine(bitmap, 70);
	drawString(bitmap, 280,  70, 8+7*hardware.L,  "L");
	drawInt(   bitmap, 300,  70, 8+7*hardware.L, 2, hardware.La);
	drawString(bitmap, 420,  70, 8+7*hardware.R,  "R");
	drawInt(   bitmap, 440,  70, 8+7*hardware.R, 2, hardware.Ra);
	drawString(bitmap, 370, 100, 8+7*hardware.S,  "S");
	drawString(bitmap, 300, 120, 10,              "D");
	drawString(bitmap, 280, 120, 8+7*hardware.Dl, "L");
	drawString(bitmap, 320, 120, 8+7*hardware.Dr, "R");
	drawString(bitmap, 300, 100, 8+7*hardware.Du, "U");
	drawString(bitmap, 300, 140, 8+7*hardware.Dd, "D");
	drawString(bitmap, 440, 130, 8+7*hardware.A,  "A");
	drawString(bitmap, 420, 135, 8+7*hardware.B,  "B");
	drawString(bitmap, 460, 125, 8+7*hardware.X,  "X");
	drawString(bitmap, 435, 110, 8+7*hardware.Y,  "Y");
	drawString(bitmap, 450,  90, 8+7*hardware.Z,  "Z");
	//Button Outputs
	eraseCharLine(bitmap, 180);
	drawString(bitmap, 280, 180, 8+7*btn.L,  "L");
	drawInt(   bitmap, 300, 180, 8+7*btn.L, 2, btn.La);
	drawString(bitmap, 420, 180, 8+7*btn.R,  "R");
	drawInt(   bitmap, 440, 180, 8+7*btn.R, 2, btn.Ra);
	drawString(bitmap, 370, 210, 8+7*btn.S,  "S");
	drawString(bitmap, 300, 230, 10,         "D");
	drawString(bitmap, 280, 230, 8+7*btn.Dl, "L");
	drawString(bitmap, 320, 230, 8+7*btn.Dr, "R");
	drawString(bitmap, 300, 210, 8+7*btn.Du, "U");
	drawString(bitmap, 300, 250, 8+7*btn.Dd, "D");
	drawString(bitmap, 440, 240, 8+7*btn.A,  "A");
	drawString(bitmap, 420, 245, 8+7*btn.B,  "B");
	drawString(bitmap, 460, 235, 8+7*btn.X,  "X");
	drawString(bitmap, 435, 220, 8+7*btn.Y,  "Y");
	drawString(bitmap, 450, 200, 8+7*btn.Z,  "Z");

	//erase the graph
	for(int y=40; y<256+40+1; y++) {
		memset(bitmap + y*VWIDTHBYTE, BLACK2, 128+1/*256 pixels = 128 bytes*/);
	}

	const int xCenter = 128;//starts at 1
	const int yCenter = 168;//starts at 40

	//octagon
	drawLine(bitmap, xCenter+  0, yCenter-100, xCenter+74, yCenter-74, 10);
	drawLine(bitmap, xCenter+100, yCenter+  0, xCenter+74, yCenter-74, 10);
	drawLine(bitmap, xCenter+100, yCenter+  0, xCenter+74, yCenter+74, 10);
	drawLine(bitmap, xCenter+  0, yCenter+100, xCenter+74, yCenter+74, 10);
	drawLine(bitmap, xCenter+  0, yCenter+100, xCenter-74, yCenter+74, 10);
	drawLine(bitmap, xCenter-100, yCenter+  0, xCenter-74, yCenter+74, 10);
	drawLine(bitmap, xCenter-100, yCenter+  0, xCenter-74, yCenter-74, 10);
	drawLine(bitmap, xCenter+  0, yCenter-100, xCenter-74, yCenter-74, 10);

	//current left stick position
	drawLine(bitmap, xCenter+btn.Ax-ORG+3, yCenter-btn.Ay+ORG+3, xCenter+btn.Ax-ORG+3, yCenter-btn.Ay+ORG-2, 15);
	drawLine(bitmap, xCenter+btn.Ax-ORG+3, yCenter-btn.Ay+ORG-3, xCenter+btn.Ax-ORG-2, yCenter-btn.Ay+ORG-3, 15);
	drawLine(bitmap, xCenter+btn.Ax-ORG-3, yCenter-btn.Ay+ORG-3, xCenter+btn.Ax-ORG-3, yCenter-btn.Ay+ORG+2, 15);
	drawLine(bitmap, xCenter+btn.Ax-ORG-3, yCenter-btn.Ay+ORG+3, xCenter+btn.Ax-ORG+2, yCenter-btn.Ay+ORG+3, 15);

	//current c-stick position
	drawLine(bitmap, xCenter+btn.Cx-ORG+1, yCenter-btn.Cy+ORG+1, xCenter+btn.Cx-ORG+1, yCenter-btn.Cy+ORG+0, 15);
	drawLine(bitmap, xCenter+btn.Cx-ORG+1, yCenter-btn.Cy+ORG-1, xCenter+btn.Cx-ORG+0, yCenter-btn.Cy+ORG-1, 15);
	drawLine(bitmap, xCenter+btn.Cx-ORG-1, yCenter-btn.Cy+ORG-1, xCenter+btn.Cx-ORG-1, yCenter-btn.Cy+ORG+0, 15);
	drawLine(bitmap, xCenter+btn.Cx-ORG-1, yCenter-btn.Cy+ORG+1, xCenter+btn.Cx-ORG+0, yCenter-btn.Cy+ORG+1, 15);

	//stick coordinates
	eraseCharLine(bitmap, 340);
	eraseCharLine(bitmap, 360);
	//left stick
	drawInt(bitmap,     20, 340, 15, 2, btn.Ax-ORG);
	drawInt(bitmap,     20, 360, 15, 2, btn.Ay-ORG);
	const int axCoord = btn.Ax - ORG;
	const int ayCoord = btn.Ay - ORG;
	float axMelee;
	float ayMelee;
	meleeCoordClamp(axCoord, ayCoord, axMelee, ayMelee);
	drawFloat(bitmap,  120, 340, 15, 0, 7, axMelee);
	drawFloat(bitmap,  120, 360, 15, 0, 7, ayMelee);
	//c-stick
	drawInt(bitmap,    280, 340, 15, 2, btn.Cx-ORG);
	drawInt(bitmap,    280, 360, 15, 2, btn.Cy-ORG);
	const int cxCoord = btn.Cx - ORG;
	const int cyCoord = btn.Cy - ORG;
	float cxMelee;
	float cyMelee;
	meleeCoordClamp(cxCoord, cyCoord, cxMelee, cyMelee);
	drawFloat(bitmap,  380, 340, 15, 0, 7, cxMelee);
	drawFloat(bitmap,  380, 360, 15, 0, 7, cyMelee);
}

void drawXYScope(unsigned char bitmap[],
		const unsigned int menu,
		const int itemIndex,
		DataCapture &capture) {
	drawString(bitmap,  20,  20, 15, MenuNames[menu]);
	drawString(bitmap, 240,  20, 15, xyscope5);

	const int xCenter = 128;//starts at 1
	const int yCenter = 168;//starts at 40
	drawString(bitmap, 280, 50, 15, xyscope1);
	if(itemIndex == 0) {
		if(capture.stickmap != 0) {
			drawString(bitmap, 280, 70, 15, arrowLeft);
		}
		if(capture.stickmap < 6) {
			drawString(bitmap, 470, 70, 15, arrowRight);
		}
	}
	switch(capture.stickmap) {
		case 0:
			drawString(bitmap, 300, 70, 15, stickmap0);
			break;
		case 1:
			drawString(bitmap, 300, 70, 15, stickmap1);
			drawImage(bitmap, deadzone_image, deadzone_indexes, 1, 40);
			break;
		case 2:
			drawString(bitmap, 300, 70, 15, stickmap2);
			drawImage(bitmap, await_image, await_indexes, 1, 40);
			break;
		case 3:
			drawString(bitmap, 300, 70, 15, stickmap3);
			drawImage(bitmap, movewait_image, movewait_indexes, 1, 40);
			break;
		case 4:
			drawString(bitmap, 300, 70, 15, stickmap4);
			drawImage(bitmap, crouch_image, crouch_indexes, 1, 40);
			break;
		case 5:
			drawString(bitmap, 300, 70, 15, stickmap5);
			drawImage(bitmap, ledgeL_image, ledgeL_indexes, 1, 40);
			break;
		case 6:
			drawString(bitmap, 300, 70, 15, stickmap6);
			drawImage(bitmap, ledgeR_image, ledgeR_indexes, 1, 40);
			break;
		default:
			break;
	}

	drawString(bitmap, 280, 100, 15, xyscope2);
	if(itemIndex == 1) {
		if(capture.captureStick == CSTICK) {
			drawString(bitmap, 280, 120, 15, arrowLeft);
		} else {
			drawString(bitmap, 410, 120, 15, arrowRight);
		}
	}
	if(capture.captureStick == ASTICK) {
		drawString(bitmap, 300, 120, 15, leftright0);
	} else {
		drawString(bitmap, 300, 120, 15, leftright1);
	}

	drawString(bitmap, 280, 150, 15, xyscope3);
	drawInt(bitmap, 290, 170, 15, 1, capture.viewIndex);
	if(itemIndex == 2) {
		if(capture.viewIndex != 0) {
			drawString(bitmap, 280, 170, 15, arrowLeft);
		}
		if(capture.viewIndex < 99) {
			drawString(bitmap, 330, 170, 15, arrowRight);
		}
	}

	if(capture.done) {
		drawString(bitmap, 280, 200, 15, xyscope4);
		if(capture.abxyszrl[capture.viewIndex] & 0b0000'0001) {
			drawString(bitmap, 280, 220, 15, "A");
		}
		if(capture.abxyszrl[capture.viewIndex] & 0b0000'0010) {
			drawString(bitmap, 300, 220, 15, "B");
		}
		if(capture.abxyszrl[capture.viewIndex] & 0b0000'0100) {
			drawString(bitmap, 320, 220, 15, "X");
		}
		if(capture.abxyszrl[capture.viewIndex] & 0b0000'1000) {
			drawString(bitmap, 340, 220, 15, "Y");
		}
		if(capture.abxyszrl[capture.viewIndex] & 0b0001'0000) {
			drawString(bitmap, 420, 220, 15, "S");
		}
		if(capture.abxyszrl[capture.viewIndex] & 0b0010'0000) {
			drawString(bitmap, 400, 220, 15, "Z");
		}
		if(capture.abxyszrl[capture.viewIndex] & 0b0100'0000) {
			drawString(bitmap, 380, 220, 15, "R");
		}
		if(capture.abxyszrl[capture.viewIndex] & 0b1000'0000) {
			drawString(bitmap, 360, 220, 15, "L");
		}

		for (int i=0; i < 100; i++) {
			const int index = (i + capture.startIndex) % 100;
			const int x = capture.a1[index]-ORG;
			const int y = capture.a2[index]-ORG;
			const int ux = capture.a1Unfilt[index]-ORG;
			const int uy = capture.a2Unfilt[index]-ORG;
			if(i != capture.viewIndex) {
				//unfiltered
				drawLine(bitmap, xCenter+ux+0, yCenter-uy+0, xCenter+ux+0, yCenter-uy-0, 13);
				//filtered
				drawLine(bitmap, xCenter+x+0, yCenter-y+0, xCenter+x+0, yCenter-y-0, 15);
			} else {
				//unfiltered
				drawLine(bitmap, xCenter+ux+1, yCenter-uy+1, xCenter+ux+1, yCenter-uy-(1-1), 13);
				drawLine(bitmap, xCenter+ux+1, yCenter-uy-1, xCenter+ux-(1-1), yCenter-uy-1, 13);
				drawLine(bitmap, xCenter+ux-1, yCenter-uy-1, xCenter+ux-1, yCenter-uy+(1-1), 13);
				drawLine(bitmap, xCenter+ux-1, yCenter-uy+1, xCenter+ux+(1-1), yCenter-uy+1, 13);
				//filtered
				drawLine(bitmap, xCenter+x+2, yCenter-y+2, xCenter+x+2, yCenter-y-(2-1), 15);
				drawLine(bitmap, xCenter+x+2, yCenter-y-2, xCenter+x-(2-1), yCenter-y-2, 15);
				drawLine(bitmap, xCenter+x-2, yCenter-y-2, xCenter+x-2, yCenter-y+(2-1), 15);
				drawLine(bitmap, xCenter+x-2, yCenter-y+2, xCenter+x+(2-1), yCenter-y+2, 15);
			}
		}

		//coordinate view
		drawString(bitmap,  30, 300, 15, xyscope6);
		drawString(bitmap,  30, 320, 15, inputview4);//reused

		//get values at the view index
		const int index = (capture.viewIndex + capture.startIndex) % 100;
		const int x = capture.a1[index]-ORG;
		const int y = capture.a2[index]-ORG;
		const int ux = capture.a1Unfilt[index]-ORG;
		const int uy = capture.a2Unfilt[index]-ORG;
		//unfiltered
		drawInt(bitmap,     20, 340, 15, 2, ux);
		drawInt(bitmap,     20, 360, 15, 2, uy);
		float uxMelee;
		float uyMelee;
		meleeCoordClamp(ux, uy, uxMelee, uyMelee);
		drawFloat(bitmap,  120, 340, 15, 0, 7, uxMelee);
		drawFloat(bitmap,  120, 360, 15, 0, 7, uyMelee);
		//filtered
		drawInt(bitmap,    280, 340, 15, 2, x);
		drawInt(bitmap,    280, 360, 15, 2, y);
		float xMelee;
		float yMelee;
		meleeCoordClamp(x, y, xMelee, yMelee);
		drawFloat(bitmap,  380, 340, 15, 0, 7, xMelee);
		drawFloat(bitmap,  380, 360, 15, 0, 7, yMelee);
	}
}

void drawTimeScope(unsigned char bitmap[],
		const unsigned int menu,
		const int itemIndex,
		DataCapture &capture) {
	drawString(bitmap,  20,  20, 15, MenuNames[menu]);
	if(itemIndex != 2) {
		drawString(bitmap, 240,  20, 15, timescope0);
	} else {
		drawString(bitmap, 240,  20, 15, xyscope5);
	}

	//which input to graph
	drawString(bitmap, 30, 300, 15, timescope1);
	if(itemIndex == 0) {
		drawString(bitmap, 20, 320, 15, arrowRight);
	}
	if(capture.mode != CM_TRIG) {
		if(capture.captureStick == ASTICK) {
			drawString(bitmap, 50, 320, 15, "A");
		} else {
			drawString(bitmap, 50, 320, 15, "C");
		}
		if(capture.whichAxis == XAXIS) {
			drawString(bitmap, 60, 320, 15, "X");
		} else {
			drawString(bitmap, 60, 320, 15, "Y");
		}
	} else {//if(capture.mode == CM_TRIG)
		if(capture.captureStick == ASTICK) {
			drawString(bitmap, 50, 320, 15, "L");
		} else {
			drawString(bitmap, 50, 320, 15, "R");
		}
	}
	
	//what to trigger upon
	drawString(bitmap, 140, 300, 15, timescope2);
	if(itemIndex == 1) {
		drawString(bitmap, 130, 320, 8+7*(capture.mode != CM_TRIG), arrowRight);
	}
	switch(capture.mode) {
		case CM_STICK_FALL:
			drawString(bitmap, 160, 320, 15, timescope3);
			break;
		case CM_STICK_RISE:
			drawString(bitmap, 160, 320, 15, timescope4);
			break;
		case CM_STICK_PIVOT:
			drawString(bitmap, 160, 320, 15, timescope5);
			break;
		case CM_TRIG:
			drawString(bitmap, 160, 320, 15, timescope6);
			break;
		default:
			break;
	}

	//which sample point to view info of
	drawString(bitmap, 290, 300, 15, xyscope3);
	drawInt(bitmap, 300, 320, 15, 2, capture.viewIndex);
	if(itemIndex == 2) {
		drawString(bitmap, 280, 320, 15, arrowRight);
	}

	//% chance of success readout (TODO)

	const int xCenter = 5;
	const int yCenter = 168;//starts at 40

	//draw axes for the graph
	drawLine(bitmap, xCenter-1, yCenter+ORG, xCenter-1, yCenter-ORG, 9);//y-axis
	switch(capture.mode) {
		case CM_STICK_FALL:
			drawLine(bitmap, xCenter, yCenter, xCenter+399, yCenter, 9);//x-axis
			drawLine(bitmap, xCenter, yCenter+23, xCenter+399, yCenter+23, 8);//deadzone -
			drawLine(bitmap, xCenter, yCenter-23, xCenter+399, yCenter-23, 8);//deadzone +
			break;
		case CM_STICK_RISE:
			drawLine(bitmap, xCenter, yCenter, xCenter+399, yCenter, 9);//x-axis
			drawLine(bitmap, xCenter, yCenter+23, xCenter+399, yCenter+23, 8);//deadzone -
			drawLine(bitmap, xCenter, yCenter-23, xCenter+399, yCenter-23, 8);//deadzone +
			drawLine(bitmap, xCenter, yCenter+64, xCenter+399, yCenter+64, 8);//dash -
			drawLine(bitmap, xCenter, yCenter-64, xCenter+399, yCenter-64, 8);//dash +
			break;
		case CM_STICK_PIVOT:
			drawLine(bitmap, xCenter, yCenter, xCenter+399, yCenter, 9);//x-axis
			drawLine(bitmap, xCenter, yCenter+64, xCenter+399, yCenter+64, 8);//dash -
			drawLine(bitmap, xCenter, yCenter-64, xCenter+399, yCenter-64, 8);//dash +
			break;
		case CM_TRIG:
			drawLine(bitmap, xCenter, yCenter+ORG, xCenter+399, yCenter+ORG, 9);//x-axis
			drawLine(bitmap, xCenter, yCenter+ORG-43, xCenter+399, yCenter+ORG-43, 8);//lightshield
			break;
		default:
			break;
	}

	int oldY = capture.a1[capture.startIndex+1 % 200] - ORG;

	//draw the actual graph
	for (int i=0; i < 200; i++) {
		const int index = (i + capture.startIndex+1) % 200;
		const int zeroY = capture.a1[index];
		const int y = zeroY-ORG;
		const int uy = capture.a1Unfilt[index]-ORG;

		//highlight trigger
		if(capture.mode == CM_TRIG) {
			drawLine(bitmap, xCenter+i*2-1, yCenter+10, xCenter+i*2, yCenter+10, 5+10*capture.abxyszrl[index]);
		}
		if(capture.mode == CM_TRIG && (zeroY >= 43)) {
			drawLine(bitmap, xCenter+i*2-1, yCenter+ORG-43, xCenter+i*2, yCenter+ORG-43, 15);
		}

		//unfiltered
		drawLine(bitmap, xCenter+i*2+0, yCenter-uy+0, xCenter+i*2+0, yCenter-uy-0, 11);
		//filtered
		drawLine(bitmap, xCenter+i*2-1+0, yCenter-oldY+0, xCenter+i*2+0, yCenter-y-0, 15);

		//highlight
		if(i == capture.viewIndex) {
			//unfiltered
			drawLine(bitmap, xCenter+i*2+1, yCenter-uy+1, xCenter+i*2+1, yCenter-uy-(1-1), 11);
			drawLine(bitmap, xCenter+i*2+1, yCenter-uy-1, xCenter+i*2-(1-1), yCenter-uy-1, 11);
			drawLine(bitmap, xCenter+i*2-1, yCenter-uy-1, xCenter+i*2-1, yCenter-uy+(1-1), 11);
			drawLine(bitmap, xCenter+i*2-1, yCenter-uy+1, xCenter+i*2+(1-1), yCenter-uy+1, 11);
			//filtered
			drawLine(bitmap, xCenter+i*2+2, yCenter-y+2, xCenter+i*2+2, yCenter-y-(2-1), 15);
			drawLine(bitmap, xCenter+i*2+2, yCenter-y-2, xCenter+i*2-(2-1), yCenter-y-2, 15);
			drawLine(bitmap, xCenter+i*2-2, yCenter-y-2, xCenter+i*2-2, yCenter-y+(2-1), 15);
			drawLine(bitmap, xCenter+i*2-2, yCenter-y+2, xCenter+i*2+(2-1), yCenter-y+2, 15);
		}

		oldY = y;
	}

	//draw percent success rates
	switch(capture.mode) {
		case CM_STICK_FALL:
			drawString(bitmap, 410,  80, 15, timescope9);
			drawFloat(bitmap,  410, 110, 15, 2, 6, capture.percents[0]);
			drawString(bitmap, 470, 110, 15, "%");
			drawString(bitmap, 410, 210, 15, timescope10);
			drawFloat(bitmap,  410, 240, 15, 2, 6, fmax(0, 100-capture.percents[0]));
			drawString(bitmap, 470, 240, 15, "%");
			break;
		case CM_STICK_RISE:
			drawString(bitmap, 410,  80, 15, timescope11);
			drawFloat(bitmap,  410, 110, 15, 2, 4, capture.percents[0]);
			drawString(bitmap, 450, 110, 15, "%");
			break;
		case CM_STICK_PIVOT:
			drawString(bitmap, 410,  80, 15, timescope14);
			drawFloat(bitmap,  410, 110, 15, 2, 4, round(capture.percents[0]));
			drawString(bitmap, 450, 110, 15, "%");
			drawString(bitmap, 410, 140, 15, timescope12);
			drawString(bitmap, 410, 160, 15, timescope13);
			drawFloat(bitmap,  410, 190, 15, 2, 4, round(capture.percents[1]));
			drawString(bitmap, 450, 190, 15, "%");
			drawString(bitmap, 410, 220, 15, timescope11);
			drawFloat(bitmap,  410, 250, 15, 2, 4, round(capture.percents[2]));
			drawString(bitmap, 450, 250, 15, "%");
			break;
		case CM_TRIG:
			drawString(bitmap, 410,  80, 15, timescope15);
			drawFloat(bitmap,  410, 110, 15, 2, 4, round(capture.percents[0]));
			drawString(bitmap, 450, 110, 15, "%");
			drawString(bitmap, 410, 150, 15, timescope16);
			drawFloat(bitmap,  410, 180, 15, 2, 4, round(capture.percents[1]));
			drawString(bitmap, 450, 180, 15, "%");
			drawString(bitmap, 410, 220, 15, timescope17);
			drawFloat(bitmap,  410, 250, 15, 2, 4, round(capture.percents[2]));
			drawString(bitmap, 450, 250, 15, "%");
			break;
		default:
			break;
	}

	//coordinate view
	drawString(bitmap,  30, 350, 15, timescope7);
	drawString(bitmap, 280, 350, 15, timescope8);

	//get values at the view index
	const int index = (capture.viewIndex + capture.startIndex+1) % 200;
	const int origin = (capture.mode == CM_TRIG) ? 0 : ORG;
	const int y = capture.a1[index]-origin;
	const int uy = capture.a1Unfilt[index]-origin;
	//filtered
	drawInt(bitmap,    140, 350, 15, 2, y);
	//unfiltered
	drawInt(bitmap,    410, 350, 15, 2, uy);
}

void drawPressSlice(unsigned char bitmap[],
		const uint8_t frame,
		DataCapture &capture) {
	const int x0 = 40 + 2*frame;
	const int x1 = x0 + 1;

	//frame boundary lines
	if(fmod(frame, 16.666667f) < 1) {
		drawLine(bitmap, x0, 145, x0, 350, 9);
		drawLine(bitmap, x1, 145, x1, 350, 9);
	}

	//button presses
	if(capture.abxyszrl[frame] & 0b0000'0001) {//A
		drawLine(bitmap, x0, 150 +  0*15, x0, 162 +  0*15, 15);
		drawLine(bitmap, x1, 150 +  0*15, x1, 162 +  0*15, 15);
	}
	if(capture.abxyszrl[frame] & 0b0000'0010) {//B
		drawLine(bitmap, x0, 150 +  1*15, x0, 162 +  1*15, 15);
		drawLine(bitmap, x1, 150 +  1*15, x1, 162 +  1*15, 15);
	}
	if(capture.abxyszrl[frame] & 0b0000'0100) {//X
		drawLine(bitmap, x0, 150 +  2*15, x0, 162 +  2*15, 15);
		drawLine(bitmap, x1, 150 +  2*15, x1, 162 +  2*15, 15);
	}
	if(capture.abxyszrl[frame] & 0b0000'1000) {//Y
		drawLine(bitmap, x0, 150 +  3*15, x0, 162 +  3*15, 15);
		drawLine(bitmap, x1, 150 +  3*15, x1, 162 +  3*15, 15);
	}
	if(capture.abxyszrl[frame] & 0b1000'0000) {//L
		drawLine(bitmap, x0, 150 +  4*15, x0, 162 +  4*15, 15);
		drawLine(bitmap, x1, 150 +  4*15, x1, 162 +  4*15, 15);
	}
	if(capture.axaycxcyrl[frame] & 0b0010'0000) {//La
		drawLine(bitmap, x0, 150 +  5*15, x0, 162 +  5*15, 15);
		drawLine(bitmap, x1, 150 +  5*15, x1, 162 +  5*15, 15);
	}
	if(capture.abxyszrl[frame] & 0b0100'0000) {//R
		drawLine(bitmap, x0, 150 +  6*15, x0, 162 +  6*15, 15);
		drawLine(bitmap, x1, 150 +  6*15, x1, 162 +  6*15, 15);
	}
	if(capture.axaycxcyrl[frame] & 0b0001'0000) {//Ra
		drawLine(bitmap, x0, 150 +  7*15, x0, 162 +  7*15, 15);
		drawLine(bitmap, x1, 150 +  7*15, x1, 162 +  7*15, 15);
	}
	if(capture.abxyszrl[frame] & 0b0010'0000) {//Z
		drawLine(bitmap, x0, 150 +  8*15, x0, 162 +  8*15, 15);
		drawLine(bitmap, x1, 150 +  8*15, x1, 162 +  8*15, 15);
	}
	if(capture.axaycxcyrl[frame] & 0b0000'0001) {//Ax
		drawLine(bitmap, x0, 150 +  9*15, x0, 162 +  9*15, 15);
		drawLine(bitmap, x1, 150 +  9*15, x1, 162 +  9*15, 15);
	}
	if(capture.axaycxcyrl[frame] & 0b0000'0010) {//Ay
		drawLine(bitmap, x0, 150 + 10*15, x0, 162 + 10*15, 15);
		drawLine(bitmap, x1, 150 + 10*15, x1, 162 + 10*15, 15);
	}
	if(capture.axaycxcyrl[frame] & 0b0000'0100) {//Cx
		drawLine(bitmap, x0, 150 + 11*15, x0, 162 + 11*15, 15);
		drawLine(bitmap, x1, 150 + 11*15, x1, 162 + 11*15, 15);
	}
	if(capture.axaycxcyrl[frame] & 0b0000'1000) {//Cy
		drawLine(bitmap, x0, 150 + 12*15, x0, 162 + 12*15, 15);
		drawLine(bitmap, x1, 150 + 12*15, x1, 162 + 12*15, 15);
	}
}

void drawPressFrames(unsigned char bitmap[],
		DataCapture &capture) {
	int8_t a  = 0;
	int8_t b  = 0;
	int8_t x  = 0;
	int8_t y  = 0;
	int8_t l  = 0;
	int8_t la = 0;
	int8_t r  = 0;
	int8_t ra = 0;
	int8_t z  = 0;
	int8_t ax = 0;
	int8_t ay = 0;
	int8_t cx = 0;
	int8_t cy = 0;

	int frame = 16;

	//was it pressed initially?
	if(capture.abxyszrl[0] & 0b0000'0001) {//A
		a = -1;
	}
	if(capture.abxyszrl[0] & 0b0000'0010) {//B
		b = -1;
	}
	if(capture.abxyszrl[0] & 0b0000'0100) {//X
		x = -1;
	}
	if(capture.abxyszrl[0] & 0b0000'1000) {//Y
		y = -1;
	}
	if(capture.abxyszrl[0] & 0b1000'0000) {//L
		l = -1;
	}
	if(capture.axaycxcyrl[0] & 0b0010'0000) {//La
		la = -1;
	}
	if(capture.abxyszrl[0] & 0b0100'0000) {//R
		r = -1;
	}
	if(capture.axaycxcyrl[0] & 0b0001'0000) {//Ra
		ra = -1;
	}
	if(capture.abxyszrl[0] & 0b0010'0000) {//Z
		z = -1;
	}
	if(capture.axaycxcyrl[0] & 0b0000'0001) {//Ax
		ax = -1;
	}
	if(capture.axaycxcyrl[0] & 0b0000'0010) {//Ay
		ay = -1;
	}
	if(capture.axaycxcyrl[0] & 0b0000'0100) {//Cx
		cx = -1;
	}
	if(capture.axaycxcyrl[0] & 0b0000'1000) {//Cy
		cy = -1;
	}

	for(int frame = 1; frame < 200; frame++) {
		if(capture.abxyszrl[frame] & 0b0000'0001) {//A
			if(a == 0) {
				a = 1;
				drawFloat(bitmap, 440, 150 +  0*15, 15, 1, 6, frame/16.666667);
			}
		} else {
			if(a == -1) {
				a = 1;
				drawFloat(bitmap, 440, 150 +  0*15, 15, 1, 6, frame/16.666667);
			}
		}
		if(capture.abxyszrl[frame] & 0b0000'0010) {//B
			if(b == 0) {
				b = 1;
				drawFloat(bitmap, 440, 150 +  1*15, 15, 1, 6, frame/16.666667);
			}
		} else {
			if(b == -1) {
				b = 1;
				drawFloat(bitmap, 440, 150 +  1*15, 15, 1, 6, frame/16.666667);
			}
		}
		if(capture.abxyszrl[frame] & 0b0000'0100) {//X
			if(x == 0) {
				x = 1;
				drawFloat(bitmap, 440, 150 +  2*15, 15, 1, 6, frame/16.666667);
			}
		} else {
			if(x == -1) {
				x = 1;
				drawFloat(bitmap, 440, 150 +  2*15, 15, 1, 6, frame/16.666667);
			}
		}
		if(capture.abxyszrl[frame] & 0b0000'1000) {//Y
			if(y == 0) {
				y = 1;
				drawFloat(bitmap, 440, 150 +  3*15, 15, 1, 6, frame/16.666667);
			}
		} else {
			if(y == -1) {
				y = 1;
				drawFloat(bitmap, 440, 150 +  3*15, 15, 1, 6, frame/16.666667);
			}
		}
		if(capture.abxyszrl[frame] & 0b1000'0000) {//L
			if(l == 0) {
				l = 1;
				drawFloat(bitmap, 440, 150 +  4*15, 15, 1, 6, frame/16.666667);
			}
		} else {
			if(l == -1) {
				l = 1;
				drawFloat(bitmap, 440, 150 +  4*15, 15, 1, 6, frame/16.666667);
			}
		}
		if(capture.axaycxcyrl[frame] & 0b0010'0000) {//La
			if(la == 0) {
				la = 1;
				drawFloat(bitmap, 440, 150 +  5*15, 15, 1, 6, frame/16.666667);
			}
		} else {
			if(la == -1) {
				la = 1;
				drawFloat(bitmap, 440, 150 +  5*15, 15, 1, 6, frame/16.666667);
			}
		}
		if(capture.abxyszrl[frame] & 0b0100'0000) {//R
			if(r == 0) {
				r = 1;
				drawFloat(bitmap, 440, 150 +  6*15, 15, 1, 6, frame/16.666667);
			}
		} else {
			if(r == -1) {
				r = 1;
				drawFloat(bitmap, 440, 150 +  6*15, 15, 1, 6, frame/16.666667);
			}
		}
		if(capture.axaycxcyrl[frame] & 0b0001'0000) {//Ra
			if(ra == 0) {
				ra = 1;
				drawFloat(bitmap, 440, 150 +  7*15, 15, 1, 6, frame/16.666667);
			}
		} else {
			if(ra == -1) {
				ra = 1;
				drawFloat(bitmap, 440, 150 +  7*15, 15, 1, 6, frame/16.666667);
			}
		}
		if(capture.abxyszrl[frame] & 0b0010'0000) {//Z
			if(z == 0) {
				z = 1;
				drawFloat(bitmap, 440, 150 +  8*15, 15, 1, 6, frame/16.666667);
			}
		} else {
			if(z == -1) {
				z = 1;
				drawFloat(bitmap, 440, 150 +  8*15, 15, 1, 6, frame/16.666667);
			}
		}
		if(capture.axaycxcyrl[frame] & 0b0000'0001) {//Ax
			if(ax == 0) {
				ax = 1;
				drawFloat(bitmap, 440, 150 +  9*15, 15, 1, 6, frame/16.666667);
			}
		} else {
			if(ax == -1) {
				ax = 1;
				drawFloat(bitmap, 440, 150 +  9*15, 15, 1, 6, frame/16.666667);
			}
		}
		if(capture.axaycxcyrl[frame] & 0b0000'0010) {//Ay
			if(ay == 0) {
				ay = 1;
				drawFloat(bitmap, 440, 150 +  10*15, 15, 1, 6, frame/16.666667);
			}
		} else {
			if(ay == -1) {
				ay = 1;
				drawFloat(bitmap, 440, 150 +  10*15, 15, 1, 6, frame/16.666667);
			}
		}
		if(capture.axaycxcyrl[frame] & 0b0000'0100) {//Cx
			if(cx == 0) {
				cx = 1;
				drawFloat(bitmap, 440, 150 +  11*15, 15, 1, 6, frame/16.666667);
			}
		} else {
			if(cx == -1) {
				cx = 1;
				drawFloat(bitmap, 440, 150 +  11*15, 15, 1, 6, frame/16.666667);
			}
		}
		if(capture.axaycxcyrl[frame] & 0b0000'1000) {//Cy
			if(cy == 0) {
				cy = 1;
				drawFloat(bitmap, 440, 150 +  12*15, 15, 1, 6, frame/16.666667);
			}
		} else {
			if(cy == -1) {
				cy = 1;
				drawFloat(bitmap, 440, 150 +  12*15, 15, 1, 6, frame/16.666667);
			}
		}
	}
}

void drawPresstime(unsigned char bitmap[],
		const unsigned int menu,
		const int itemIndex,
		DataCapture &capture) {
	drawString(bitmap,  20,  20, 15, MenuNames[menu]);
	drawString(bitmap,  30,  50, 15, presstime1);
	drawString(bitmap,  30,  70, 15, presstime2);
	drawString(bitmap,  30,  90, 15, reaction3);
	drawString(bitmap,  30, 120, 15, reaction4);
	drawInt(   bitmap,  90, 120, 15, 2, capture.stickThresh);
	drawString(bitmap, 170, 120, 15, reaction5);
	drawInt(   bitmap, 250, 120, 15, 2, capture.triggerThresh);
	drawString(bitmap, 330, 120, 15, presstime4);
	if(capture.autoRepeat) {
		drawString(bitmap, 460, 120, 15, presstime5);
	} else {
		drawString(bitmap, 460, 120, 15, presstime6);
	}
	if(itemIndex == 0) {
		drawString(bitmap,  10, 120, 15, arrowRight);
	} else if (itemIndex == 1) {
		drawString(bitmap, 150, 120, 15, arrowRight);
	} else {
		drawString(bitmap, 310, 120, 15, arrowRight);
	}

	drawString(bitmap,  10, 150 +  0*15, 15, "A");
	drawString(bitmap,  10, 150 +  1*15, 15, "B");
	drawString(bitmap,  10, 150 +  2*15, 15, "X");
	drawString(bitmap,  10, 150 +  3*15, 15, "Y");
	drawString(bitmap,  10, 150 +  4*15, 15, "L");
	drawString(bitmap,  10, 150 +  5*15, 15, "La");
	drawString(bitmap,  10, 150 +  6*15, 15, "R");
	drawString(bitmap,  10, 150 +  7*15, 15, "Ra");
	drawString(bitmap,  10, 150 +  8*15, 15, "Z");
	drawString(bitmap,  10, 150 +  9*15, 15, "AX");
	drawString(bitmap,  10, 150 + 10*15, 15, "AY");
	drawString(bitmap,  10, 150 + 11*15, 15, "CX");
	drawString(bitmap,  10, 150 + 12*15, 15, "CY");

	for(int frame = 0; frame < 200; frame++) {
		drawPressSlice(bitmap, frame, capture);
	}

	drawPressFrames(bitmap, capture);

	if(capture.begin == false && capture.done == false) {
		drawString(bitmap, 30, 360, 15, presstime3);
	}
}

//You wait a random amount of time before actually calling this draw function
//Then the draw function, as soon as it is done, initiates recording
void drawReaction(unsigned char bitmap[],
		const unsigned int menu,
		const int itemIndex,
		DataCapture &capture) {
	drawString(bitmap,  20,  20, 15, MenuNames[menu]);
	drawString(bitmap,  30,  50, 15, reaction1);
	drawString(bitmap,  30,  70, 15, reaction2);
	drawString(bitmap,  30,  90, 15, reaction3);
	drawString(bitmap,  30, 120, 15, reaction4);
	drawInt(   bitmap, 160, 120, 15, 0, capture.stickThresh);
	drawString(bitmap, 280, 120, 15, reaction5);
	drawInt(   bitmap, 410, 120, 15, 0, capture.triggerThresh);
	if(itemIndex == 0) {
		drawString(bitmap,  10, 120, 15, arrowRight);
	} else {
		drawString(bitmap, 260, 120, 15, arrowRight);
	}
	if(!capture.done) {
		//draw white square
		for(int i = 0; i < 50; i++) {
			memset(bitmap + (180+i)*VWIDTHBYTE + 128 - 12, WHITE2, 25/*50 pixels wide*/);
		}
		//start capture
		capture.mode = CM_REACTION;
	} else {
		//write the reaction time to the screen
		drawString(bitmap,  30, 300, 15, reaction6);
		drawInt(   bitmap,  60, 300, 15, 0, capture.delay);
		drawString(bitmap, 280, 300, 15, reaction7);
		drawFloat( bitmap, 350, 300, 15, 1, 5, capture.delay/16.667f);
	}
}

void drawVision(unsigned char bitmap[],
		const unsigned int menu,
		const int itemIndex,
		const bool changeMade,
		const Buttons btn,
		const RawStick raw,
		const ControlConfig &controls,
		const StickParams &aStick,
		const StickParams &cStick) {
	drawString(bitmap,  20,  20, 15, MenuNames[menu]);
	if(changeMade) {
		drawString(bitmap, 300, 20, 15, bToSave);
	}
	drawString(bitmap,  30,  50, 15, vision1);
	drawString(bitmap,  30,  70, 15, vision2);
	drawString(bitmap,  30, 100, 15, vision3);
	drawInt(bitmap,    220, 100, 15, 2, controls.interlaceOffset);
	drawLine(bitmap, 500, 1, 1, 101, 15);
	drawLine(bitmap, 500, 101, 1, 201, 15);
	drawLine(bitmap, 500, 201, 1, 301, 15);
	drawLine(bitmap, 500, 301, 1, 380, 15);
}

void drawMenuFast(unsigned char bitmap[],
		const unsigned int menu,
		const int itemIndex,
		const bool changeMade,
		const int currentCalStep,
		const Buttons btn,
		const Buttons hardware,
		const RawStick raw,
		const ControlConfig &controls,
		const StickParams &aStick,
		const StickParams &cStick) {
	if (MenuIndex[menu][1] <= 6) {
		drawString(bitmap, 20, 20, 15, MenuNames[menu]);
		for(int i = 0; i < MenuIndex[menu][1]; i++) {
			drawString(bitmap, 50, 80 + 30*i, 15, MenuNames[MenuIndex[menu][i+2]]);
		}
		drawString(bitmap, 20, 80 + 30*itemIndex, 15, arrowRight);
	}
	switch(menu) {
		case MENU_ASTICKCAL:
			drawStickCalFast(bitmap, menu, currentCalStep, changeMade, ASTICK, btn, raw, controls, aStick, cStick);
			break;
		case MENU_CSTICKCAL:
			drawStickCalFast(bitmap, menu, currentCalStep, changeMade, CSTICK, btn, raw, controls, aStick, cStick);
			break;
		case MENU_STICKDBG:
			drawStickdbgFast(bitmap, menu, itemIndex, changeMade, btn, raw, controls, aStick, cStick);
			break;
		case MENU_TRIGGER:
		case MENU_LTRIGGER:
		case MENU_RTRIGGER:
			drawTriggerFast(bitmap, menu, itemIndex, changeMade, btn, hardware, raw, controls, aStick, cStick);
			break;
		case MENU_INPUTVIEW:
			drawInputviewFast(bitmap, menu, itemIndex, changeMade, btn, hardware, raw, controls, aStick, cStick);
			break;
		default:
			break;
	}
}

void drawMenu(unsigned char bitmap[],
		const unsigned int menu,
		const int itemIndex,
		const bool changeMade,
		const int currentCalStep,
		const int version,
		const Buttons btn,
		const RawStick raw,
		const ControlConfig &controls,
		const StickParams &aStick,
		const StickParams &cStick,
		DataCapture &capture) {
	//Basic menus
	if(MenuIndex[menu][1] == 0) {
		drawImage(bitmap, Cute_Ghost, Cute_Ghost_Index, VWIDTH/2-112, 0);//224x300
		drawString(bitmap, VWIDTH/2-105, 320, 15, splashWelcome);
		drawString(bitmap, VWIDTH/2- 70, 340, 15, splashPress);
		drawString(bitmap,           10, 340, 15, "v0.");
		drawInt(   bitmap,           30, 340, 15, 0, abs(version));
		if(version < 0) {
			if(version > -100) {
				drawString(bitmap,   70, 340, 15, "Beta");
			} else {//handle versions up to 9999
				drawString(bitmap,   80, 340, 15, "Beta");
			}
		}
	} else if (MenuIndex[menu][1] <= 6) {
		drawString(bitmap, 20, 20, 15, MenuNames[menu]);
		for(int i = 0; i < MenuIndex[menu][1]; i++) {
			drawString(bitmap, 50, 80 + 30*i, 15, MenuNames[MenuIndex[menu][i+2]]);
		}
		drawString(bitmap, 20, 80 + 30*itemIndex, 15, arrowRight);
	} else {
		//placeholder for other screens that don't need menu graphics drawn
	}

	//big switch case to draw bottom level pages
	// and additional graphics on other menus
	switch(menu) {
		case MENU_ASTICKCAL:
			drawStickCal(bitmap, menu, currentCalStep, changeMade, ASTICK, btn, raw, controls, aStick, cStick);
			break;
		case MENU_CSTICKCAL:
			drawStickCal(bitmap, menu, currentCalStep, changeMade, CSTICK, btn, raw, controls, aStick, cStick);
			break;
		case MENU_AUTOINIT:
			drawAutoinit(bitmap, menu, itemIndex, changeMade, btn, raw, controls, aStick, cStick);
			break;
		case MENU_STICKDBG:
			drawStickdbg(bitmap, menu, itemIndex, changeMade, btn, raw, controls, aStick, cStick);
			break;
		case MENU_ASNAPBACK:
			drawAsnapback(bitmap, menu, itemIndex, changeMade, btn, raw, controls, aStick, cStick);
			break;
		case MENU_AWAVE:
			drawAwave(bitmap, menu, itemIndex, changeMade, btn, raw, controls, aStick, cStick);
			break;
		case MENU_ASMOOTH:
			drawAsmooth(bitmap, menu, itemIndex, changeMade, btn, raw, controls, aStick, cStick);
			break;
		case MENU_CSNAPBACK:
			drawCsnapback(bitmap, menu, itemIndex, changeMade, btn, raw, controls, aStick, cStick);
			break;
		case MENU_CWAVE:
			drawCwave(bitmap, menu, itemIndex, changeMade, btn, raw, controls, aStick, cStick);
			break;
			/*
		case MENU_COFFSET:
			drawCoffset(bitmap, menu, itemIndex, changeMade, btn, raw, controls, aStick, cStick);
			break;
			*/
		case MENU_CARDINALS:
			drawCardinals(bitmap, menu, itemIndex, changeMade, controls);
			break;
		case MENU_RADIUS:
			drawRadius(bitmap, menu, itemIndex, changeMade, controls);
			break;
		case MENU_SET_OVER:
			drawSet_over(bitmap, menu, itemIndex, changeMade, btn, raw, controls, aStick, cStick);
			break;
		case MENU_REMAP:
			drawRemap(bitmap, menu, itemIndex, changeMade, btn, raw, controls, aStick, cStick);
			break;
		case MENU_RUMBLE:
			drawRumble(bitmap, menu, itemIndex, changeMade, btn, raw, controls, aStick, cStick);
			break;
		case MENU_TRIGGER:
			drawTrigger(bitmap, menu, itemIndex, changeMade, btn, raw, controls, aStick, cStick);
			break;
		case MENU_LTRIGGER:
			drawLtrigger(bitmap, menu, itemIndex, changeMade, btn, raw, controls, aStick, cStick);
			break;
		case MENU_RTRIGGER:
			drawRtrigger(bitmap, menu, itemIndex, changeMade, btn, raw, controls, aStick, cStick);
			break;
		case MENU_TOURNEY:
			drawTourney(bitmap, menu, changeMade, controls);
			break;
		case MENU_RESET:
			drawReset(bitmap, menu, itemIndex, changeMade, btn, raw, controls, aStick, cStick);
			break;
		case MENU_INPUTVIEW:
			drawInputview(bitmap, menu, itemIndex, changeMade, btn, raw, controls, aStick, cStick);
			break;
		case MENU_XYSCOPE:
			drawXYScope(bitmap, menu, itemIndex, capture);
			break;
		case MENU_TIMESCOPE:
			drawTimeScope(bitmap, menu, itemIndex, capture);
			break;
		case MENU_PRESSTIME:
			drawPresstime(bitmap, menu, itemIndex, capture);
			break;
		case MENU_REACTION:
			drawReaction(bitmap, menu, itemIndex, capture);
			break;
		case MENU_VISION:
			drawVision(bitmap, menu, itemIndex, changeMade, btn, raw, controls, aStick, cStick);
			break;
		default:
			//placeholder for screens that don't have anything defined
			if(MenuIndex[menu][1] > 6) {
				drawString(bitmap, 20, 20, 15, MenuNames[menu]);
				drawString2x(bitmap, 80, 180, 15, "Under Construction");
			}
	}
}
