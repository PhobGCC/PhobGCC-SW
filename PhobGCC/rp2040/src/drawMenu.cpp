#include <cmath>
#include "pico/platform.h"
#include "cvideo.h"
#include "menu.h"

#include "menuStrings.h"
#include "images/cuteGhost.h"

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
	} else if(itemIndex == 1) {
		//fit coefficients
		drawString(bitmap,  30,  50, 15, stickdbgAXfit);
		drawFloat(bitmap,   30,  70, 15, 0, 10, aStick.fitCoeffsX[0]);
		drawFloat(bitmap,   30,  90, 15, 0, 10, aStick.fitCoeffsX[0]);
		drawFloat(bitmap,   30, 110, 15, 0, 10, aStick.fitCoeffsX[0]);
		drawFloat(bitmap,   30, 130, 15, 0, 10, aStick.fitCoeffsX[0]);
		drawString(bitmap,  30, 150, 15, stickdbgAYfit);
		drawFloat(bitmap,   30, 170, 15, 0, 10, aStick.fitCoeffsY[0]);
		drawFloat(bitmap,   30, 190, 15, 0, 10, aStick.fitCoeffsY[0]);
		drawFloat(bitmap,   30, 210, 15, 0, 10, aStick.fitCoeffsY[0]);
		drawFloat(bitmap,   30, 230, 15, 0, 10, aStick.fitCoeffsY[0]);
		drawString(bitmap, 280,  50, 15, stickdbgCXfit);
		drawFloat(bitmap,  280,  70, 15, 0, 10, cStick.fitCoeffsX[0]);
		drawFloat(bitmap,  280,  90, 15, 0, 10, cStick.fitCoeffsX[0]);
		drawFloat(bitmap,  280, 110, 15, 0, 10, cStick.fitCoeffsX[0]);
		drawFloat(bitmap,  280, 130, 15, 0, 10, cStick.fitCoeffsX[0]);
		drawString(bitmap, 280, 150, 15, stickdbgCYfit);
		drawFloat(bitmap,  280, 170, 15, 0, 10, cStick.fitCoeffsY[0]);
		drawFloat(bitmap,  280, 190, 15, 0, 10, cStick.fitCoeffsY[0]);
		drawFloat(bitmap,  280, 210, 15, 0, 10, cStick.fitCoeffsY[0]);
		drawFloat(bitmap,  280, 230, 15, 0, 10, cStick.fitCoeffsY[0]);
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
	drawString(bitmap, 150, 130, 15, "CX OF:");
	drawString(bitmap, 150, 150, 15, "CY OF:");
	drawInt(bitmap,    210,  50, 15, 2, controls.cxSmoothing);
	drawInt(bitmap,    210,  70, 15, 2, controls.cySmoothing);
	drawInt(bitmap,    210,  90, 15, 2, controls.cxWaveshaping);
	drawInt(bitmap,    210, 110, 15, 2, controls.cyWaveshaping);
	drawInt(bitmap,    210, 130, 15, 2, controls.cXOffset);
	drawInt(bitmap,    210, 150, 15, 2, controls.cYOffset);
	drawString(bitmap, 280,  50, 15, "L Mode:");
	drawString(bitmap, 280,  70, 15, "R Mode:");
	drawString(bitmap, 280,  90, 15, "L Val:");
	drawString(bitmap, 280, 110, 15, "R Val:");
	drawString(bitmap, 280, 130, 15, "L WS:");
	drawString(bitmap, 280, 150, 15, "L WS:");
	drawInt(bitmap,    350,  50, 15, 2, controls.lConfig+1);
	drawInt(bitmap,    350,  70, 15, 2, controls.rConfig+1);
	drawInt(bitmap,    350,  90, 15, 2, controls.lTriggerOffset);
	drawInt(bitmap,    350, 110, 15, 2, controls.rTriggerOffset);
	drawInt(bitmap,    350, 130, 15, 2, -1);//controls.rTriggerWaveshaping);
	drawInt(bitmap,    350, 150, 15, 2, -1);//controls.rTriggerWaveshaping);
	drawString(bitmap,  30, 170, 15, "Rumble:");
	drawInt(bitmap,    110, 170, 15, 1, controls.rumble);
	if(controls.autoInit) {
		drawString(bitmap, 30, 190, 15, set_overAutoOn);
	} else {
		drawString(bitmap, 30, 190, 15, set_overAutoOff);
	}
	switch(controls.jumpConfig) {
		case DEFAULTJUMP:
			drawString(bitmap, 30, 210, 15, set_overJumpDf);
			break;
		case SWAP_XZ:
			drawString(bitmap, 30, 210, 15, set_overJumpXZ);
			break;
		case SWAP_YZ:
			drawString(bitmap, 30, 210, 15, set_overJumpYZ);
			break;
		case SWAP_XL:
			drawString(bitmap, 30, 210, 15, set_overJumpXL);
			break;
		case SWAP_XR:
			drawString(bitmap, 30, 210, 15, set_overJumpXR);
			break;
		case SWAP_YL:
			drawString(bitmap, 30, 210, 15, set_overJumpYL);
			break;
		case SWAP_YR:
			drawString(bitmap, 30, 210, 15, set_overJumpYR);
			break;
		default:
			drawString(bitmap, 30, 210, 15, set_overJumpBr);
			break;
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
	drawString(bitmap,  30, 160, 15, leftStickX);
	drawInt(   bitmap, 160, 160, 15, 1, controls.xSnapback);
	drawString(bitmap, 280, 160, 15, leftStickY);
	drawInt(   bitmap, 410, 160, 15, 1, controls.ySnapback);
	if(itemIndex == 0) {
		drawString(bitmap,  10, 160, 15, arrowPointer);
	} else {
		drawString(bitmap, 260, 160, 15, arrowPointer);
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
		drawString(bitmap,  10, 160, 15, arrowPointer);
	} else {
		drawString(bitmap, 260, 160, 15, arrowPointer);
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
		drawString(bitmap,  10, 160, 15, arrowPointer);
	} else {
		drawString(bitmap, 260, 160, 15, arrowPointer);
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
				drawString(bitmap,  10, 160, 15, arrowPointer);
			} else {
				drawString(bitmap, 260, 160, 15, arrowPointer);
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
		drawString(bitmap,  10, 160, 15, arrowPointer);
	} else {
		drawString(bitmap, 260, 160, 15, arrowPointer);
	}
	//graph?
}

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
		drawString(bitmap,  10, 160, 15, arrowPointer);
	} else {
		drawString(bitmap, 260, 160, 15, arrowPointer);
	}
	//graph?
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
	drawString(bitmap,  30, 160, 15, currentSetting);
	drawInt(   bitmap, 190, 160, 15, 0, controls.rumble);
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
		drawString(bitmap,  10, 100, 15, arrowPointer);
	} else {
		drawString(bitmap, 260, 100, 15, arrowPointer);
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
		drawString(bitmap,  10, 100, 15, arrowPointer);
	} else {
		drawString(bitmap, 260, 100, 15, arrowPointer);
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

void drawMenuFast(unsigned char bitmap[],
		const unsigned int menu,
		const int itemIndex,
		const bool changeMade,
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
		drawString(bitmap, 20, 80 + 30*itemIndex, 15, arrowPointer);
	}
	switch(menu) {
		case MENU_STICKDBG:
			drawStickdbgFast(bitmap, menu, itemIndex, changeMade, btn, raw, controls, aStick, cStick);
			break;
		case MENU_TRIGGER:
		case MENU_LTRIGGER:
		case MENU_RTRIGGER:
			drawTriggerFast(bitmap, menu, itemIndex, changeMade, btn, hardware, raw, controls, aStick, cStick);
			break;
		default:
			break;
	}
}

void drawMenu(unsigned char bitmap[],
		const unsigned int menu,
		const int itemIndex,
		const bool changeMade,
		const Buttons btn,
		const RawStick raw,
		const ControlConfig &controls,
		const StickParams &aStick,
		const StickParams &cStick) {
	//Basic menus
	if(MenuIndex[menu][1] == 0) {
		drawImage(bitmap, Cute_Ghost, Cute_Ghost_Index, VWIDTH/2-112, 0);//224x300
		drawString(bitmap, VWIDTH/2-105, 320, 15, splashWelcome);
		drawString(bitmap, VWIDTH/2- 70, 340, 15, splashPress);
	} else if (MenuIndex[menu][1] <= 6) {
		drawString(bitmap, 20, 20, 15, MenuNames[menu]);
		for(int i = 0; i < MenuIndex[menu][1]; i++) {
			drawString(bitmap, 50, 80 + 30*i, 15, MenuNames[MenuIndex[menu][i+2]]);
		}
		drawString(bitmap, 20, 80 + 30*itemIndex, 15, arrowPointer);
	} else {
		//placeholder for other screens that don't need menu graphics drawn
	}

	//big switch case to draw bottom level pages
	// and additional graphics on other menus
	switch(menu) {
		case MENU_ASTICKCAL:
			//left stick calibration
			drawString(bitmap,  20,  20, 15, MenuNames[menu]);
			//we need to display different text depending on the cal step. We use itemIndex to represent this.
		case MENU_STICKDBG:
			drawStickdbg(bitmap, menu, itemIndex, changeMade, btn, raw, controls, aStick, cStick);
			break;
		case MENU_SET_OVER:
			drawSet_over(bitmap, menu, itemIndex, changeMade, btn, raw, controls, aStick, cStick);
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
		case MENU_COFFSET:
			drawCoffset(bitmap, menu, itemIndex, changeMade, btn, raw, controls, aStick, cStick);
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
		default:
			//placeholder for screens that don't have anything defined
			if(MenuIndex[menu][1] > 6) {
				drawString(bitmap, 20, 20, 15, MenuNames[menu]);
			}
	}
}
