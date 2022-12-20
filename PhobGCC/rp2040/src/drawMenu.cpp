#include <cmath>
#include "pico/platform.h"
#include "cvideo.h"
#include "menu.h"

#include "images/cuteGhost.h"

void __time_critical_func(drawMenu)(unsigned char bitmap[],
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

	//big switch case to draw bottom level pages
	// and additional graphics on other menus
	switch(menu) {
		case MENU_ASTICKCAL:
			//left stick calibration
			drawString(bitmap,  20,  20, 15, MenuNames[menu]);
			//we need to display different text depending on the cal step. We use itemIndex to represent this.
		case MENU_STICKDBG:
			drawString(bitmap,  20,  20, 15, "Stick Debug Info      Press A to cycle");
			if(itemIndex == 0) {
				//raw output
				drawString(bitmap,  30,  50, 15, "A raw values");
				drawString(bitmap, 280,  50, 15, "C raw values");
				drawFloat(bitmap,   30,  70, 15, 0, 6, raw.axRaw);
				drawFloat(bitmap,  280,  70, 15, 0, 6, raw.cxRaw);
				drawFloat(bitmap,   30,  90, 15, 0, 6, raw.ayRaw);
				drawFloat(bitmap,  280,  90, 15, 0, 6, raw.cyRaw);
				drawString(bitmap,  30, 120, 15, "A linearized values");
				drawString(bitmap, 280, 120, 15, "C linearized values");
				drawFloat(bitmap,   30, 140, 15, 2, 6, raw.axLinearized);
				drawFloat(bitmap,  280, 140, 15, 2, 6, raw.cxLinearized);
				drawFloat(bitmap,   30, 160, 15, 2, 6, raw.ayLinearized);
				drawFloat(bitmap,  280, 160, 15, 2, 6, raw.cyLinearized);
				drawString(bitmap,  30, 190, 15, "A remapped values");
				drawString(bitmap, 280, 190, 15, "C remapped values");
				drawFloat(bitmap,   30, 210, 15, 2, 6, raw.axUnfiltered);
				drawFloat(bitmap,  280, 210, 15, 2, 6, raw.cxUnfiltered);
				drawFloat(bitmap,   30, 230, 15, 2, 6, raw.ayUnfiltered);
				drawFloat(bitmap,  280, 230, 15, 2, 6, raw.cyUnfiltered);
			} else if(itemIndex == 1) {
				//fit coefficients
				drawString(bitmap,  30,  50, 15, "A fit coeffs X");
				drawFloat(bitmap,   30,  70, 15, 0, 10, aStick.fitCoeffsX[0]);
				drawFloat(bitmap,   30,  90, 15, 0, 10, aStick.fitCoeffsX[0]);
				drawFloat(bitmap,   30, 110, 15, 0, 10, aStick.fitCoeffsX[0]);
				drawFloat(bitmap,   30, 130, 15, 0, 10, aStick.fitCoeffsX[0]);
				drawString(bitmap,  30, 150, 15, "A fit coeffs Y");
				drawFloat(bitmap,   30, 170, 15, 0, 10, aStick.fitCoeffsY[0]);
				drawFloat(bitmap,   30, 190, 15, 0, 10, aStick.fitCoeffsY[0]);
				drawFloat(bitmap,   30, 210, 15, 0, 10, aStick.fitCoeffsY[0]);
				drawFloat(bitmap,   30, 230, 15, 0, 10, aStick.fitCoeffsY[0]);
				drawString(bitmap, 280,  50, 15, "C fit coeffs X");
				drawFloat(bitmap,  280,  70, 15, 0, 10, cStick.fitCoeffsX[0]);
				drawFloat(bitmap,  280,  90, 15, 0, 10, cStick.fitCoeffsX[0]);
				drawFloat(bitmap,  280, 110, 15, 0, 10, cStick.fitCoeffsX[0]);
				drawFloat(bitmap,  280, 130, 15, 0, 10, cStick.fitCoeffsX[0]);
				drawString(bitmap, 280, 150, 15, "C fit coeffs Y");
				drawFloat(bitmap,  280, 170, 15, 0, 10, cStick.fitCoeffsY[0]);
				drawFloat(bitmap,  280, 190, 15, 0, 10, cStick.fitCoeffsY[0]);
				drawFloat(bitmap,  280, 210, 15, 0, 10, cStick.fitCoeffsY[0]);
				drawFloat(bitmap,  280, 230, 15, 0, 10, cStick.fitCoeffsY[0]);
			} else if(itemIndex == 2) {
				//affine coefficients
				drawString(bitmap,  30,  50, 15, "A affine transform");
				drawString(bitmap, 280,  50, 15, "C affine transform");
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
				drawString(bitmap,  30,  50, 15, "A bounds angles");
				drawString(bitmap, 280,  50, 15, "C bounds angles");
				for(int i = 0; i < 16; i++) {
					drawFloat(bitmap,  30, 70+16*i, 15, 2, 7, aStick.boundaryAngles[i]*180/M_PI);
					drawFloat(bitmap, 280, 70+16*i, 15, 2, 7, cStick.boundaryAngles[i]*180/M_PI);
					//ends at y = 262
				}
			}
			break;
		case MENU_SET_OVER:
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
		case MENU_ASNAPBACK:
			drawString(bitmap,  20,  20, 15, MenuNames[menu]);
			if(changeMade) {
				drawString(bitmap, 300, 20, 15, "Press B to save");
			}
			//                                      100       200       300       400       500 //510 is the last char
			drawString(bitmap,  30,  50, 15, "Dpad L/R selects setting, U/D changes setting.");
			drawString(bitmap,  30,  70, 15, "Set so that snapback < 23 for Melee.");
			drawString(bitmap,  30,  90, 15, "0 disables the filter. >0 shortens rise time.");
			drawString(bitmap,  30, 110, 15, "Higher makes the stick return to center slower.");
			drawString(bitmap,  30, 130, 15, "Min: 0  Max: 10  Default: 4");
			drawString(bitmap,  30, 160, 15, "Left stick X:");
			drawInt(   bitmap, 160, 160, 15, 1, controls.xSnapback);
			drawString(bitmap, 280, 160, 15, "Left stick Y:");
			drawInt(   bitmap, 410, 160, 15, 1, controls.ySnapback);
			if(itemIndex == 0) {
				drawString(bitmap,  10, 160, 15, ">");
			} else {
				drawString(bitmap, 260, 160, 15, ">");
			}
			//graph?
			break;
		case MENU_AWAVE:
			drawString(bitmap,  20,  20, 15, MenuNames[menu]);
			if(changeMade) {
				drawString(bitmap, 300, 20, 15, "Press B to save");
			}
			//                                      100       200       300       400       500 //510 is the last char
			drawString(bitmap,  30,  50, 15, "Dpad L/R selects setting, U/D changes setting.");
			drawString(bitmap,  30,  70, 15, "This makes the stick stop during fast inputs.");
			drawString(bitmap,  30,  90, 15, "Higher values reduce the speed threshold.");
			drawString(bitmap,  30, 110, 15, "It affects pivots, DBOOC, dashback, and more.");
			drawString(bitmap,  30, 130, 15, "Min: 0  Max: 15  Default: 0");
			drawString(bitmap,  30, 160, 15, "Left stick X:");
			drawInt(   bitmap, 160, 160, 15, 1, controls.axWaveshaping);
			drawString(bitmap, 280, 160, 15, "Left stick Y:");
			drawInt(   bitmap, 410, 160, 15, 1, controls.ayWaveshaping);
			if(itemIndex == 0) {
				drawString(bitmap,  10, 160, 15, ">");
			} else {
				drawString(bitmap, 260, 160, 15, ">");
			}
			//graph?
			break;
		case MENU_ASMOOTH:
			drawString(bitmap,  20,  20, 15, MenuNames[menu]);
			if(changeMade) {
				drawString(bitmap, 300, 20, 15, "Press B to save");
			}
			//                                      100       200       300       400       500 //510 is the last char
			drawString(bitmap,  30,  50, 15, "Dpad L/R selects setting, U/D changes setting.");
			drawString(bitmap,  30,  70, 15, "This averages the stick position with the past.");
			drawString(bitmap,  30,  90, 15, "It can be used to make one axis lag the other.");
			drawString(bitmap,  30, 110, 15, "Use X=5 to make down and in ledgedashes work.");
			drawString(bitmap,  30, 130, 15, "Min: 0  Max: 9  Default: 0");
			drawString(bitmap,  30, 160, 15, "Left stick X:");
			drawInt(   bitmap, 160, 160, 15, 0, controls.axSmoothing);
			drawString(bitmap, 280, 160, 15, "Left stick Y:");
			drawInt(   bitmap, 410, 160, 15, 0, controls.aySmoothing);
			if(itemIndex == 0) {
				drawString(bitmap,  10, 160, 15, ">");
			} else {
				drawString(bitmap, 260, 160, 15, ">");
			}
			//graph?
			break;
		case MENU_CSNAPBACK:
			drawString(bitmap,  20,  20, 15, MenuNames[menu]);
			if(changeMade) {
				drawString(bitmap, 300, 20, 15, "Press B to save");
			}
			//                                      100       200       300       400       500 //510 is the last char
			drawString(bitmap,  30,  50, 15, "Dpad L/R selects setting, U/D changes setting.");
			drawString(bitmap,  30,  70, 15, "This averages the stick position with the past.");
			drawString(bitmap,  30,  90, 15, "Make them 5 or 6 to reduce snapback.");
			drawString(bitmap,  30, 110, 15, "Make one axis higher to make it less responsive.");
			drawString(bitmap,  30, 130, 15, "Min: 0  Max: 9  Default: 0");
			drawString(bitmap,  30, 160, 15, "Right stick X:");
			drawInt(   bitmap, 170, 160, 15, 0, controls.cxSmoothing);
			drawString(bitmap, 280, 160, 15, "Right stick Y:");
			drawInt(   bitmap, 420, 160, 15, 0, controls.cySmoothing);
			if(itemIndex == 0) {
				drawString(bitmap,  10, 160, 15, ">");
			} else {
				drawString(bitmap, 260, 160, 15, ">");
			}
			//graph?
			break;
		case MENU_CWAVE:
			drawString(bitmap,  20,  20, 15, MenuNames[menu]);
			if(changeMade) {
				drawString(bitmap, 300, 20, 15, "Press B to save");
			}
			//                                      100       200       300       400       500 //510 is the last char
			drawString(bitmap,  30,  50, 15, "Dpad L/R selects setting, U/D changes setting.");
			drawString(bitmap,  30,  70, 15, "This makes the stick stop during fast inputs.");
			drawString(bitmap,  30,  90, 15, "Higher values reduce the speed threshold.");
			drawString(bitmap,  30, 110, 15, "We're not sure what it can be used for here.");
			drawString(bitmap,  30, 130, 15, "Min: 0  Max: 15  Default: 0");
			drawString(bitmap,  30, 160, 15, "Right stick X:");
			drawInt(   bitmap, 170, 160, 15, 1, controls.cxWaveshaping);
			drawString(bitmap, 280, 160, 15, "Right stick Y:");
			drawInt(   bitmap, 420, 160, 15, 1, controls.cyWaveshaping);
			if(itemIndex == 0) {
				drawString(bitmap,  10, 160, 15, ">");
			} else {
				drawString(bitmap, 260, 160, 15, ">");
			}
			//graph?
			break;
		case MENU_COFFSET:
			drawString(bitmap,  20,  20, 15, MenuNames[menu]);
			if(changeMade) {
				drawString(bitmap, 300, 20, 15, "Press B to save");
			}
			//                                      100       200       300       400       500 //510 is the last char
			drawString(bitmap,  30,  50, 15, "Dpad L/R selects setting, U/D changes setting.");
			drawString(bitmap,  30,  70, 15, "This changes the default origin of the C-Stick.");
			drawString(bitmap,  30,  90, 15, "This was for down-angled fsmash but it's better");
			drawString(bitmap,  30, 110, 15, " done by notch cal so this might be removed.");
			drawString(bitmap,  30, 130, 15, "Min: -127  Max: 127  Default: 0");
			drawString(bitmap,  30, 160, 15, "Right stick X:");
			drawInt(   bitmap, 170, 160, 15, 2, controls.cXOffset);
			drawString(bitmap, 280, 160, 15, "Right stick Y:");
			drawInt(   bitmap, 420, 160, 15, 2, controls.cYOffset);
			if(itemIndex == 0) {
				drawString(bitmap,  10, 160, 15, ">");
			} else {
				drawString(bitmap, 260, 160, 15, ">");
			}
			//graph?
			break;
		case MENU_REMAP:
			drawString(bitmap,  20,  20, 15, MenuNames[menu]);
			if(changeMade) {
				drawString(bitmap, 300, 20, 15, "Press B to save");
			}
			//                                      100       200       300       400       500 //510 is the last char
			drawString(bitmap,  30,  50, 15, "Dpad U/D changes setting.");
			drawString(bitmap,  30,  70, 15, "This swaps one of L, R, or Z with either X or Y.");
			drawString(bitmap,  30,  90, 15, "When L or R are swapped, they are digital only.");
			drawString(bitmap,  30, 110, 15, "Use Trigger Mode 5 to get an analog trigger");
			drawString(bitmap,  30, 130, 15, " value on a face button for lightshielding.");
			drawString(bitmap,  30, 160, 15, "Current setting:");
			switch(controls.jumpConfig) {
				case DEFAULTJUMP:
					drawString(bitmap, 200, 160, 15, "Default");
					break;
				case SWAP_XZ:
					drawString(bitmap, 200, 160, 15, "Swap X and Z");
					break;
				case SWAP_YZ:
					drawString(bitmap, 200, 160, 15, "Swap Y and Z");
					break;
				case SWAP_XL:
					drawString(bitmap, 200, 160, 15, "Swap X and L");
					break;
				case SWAP_YL:
					drawString(bitmap, 200, 160, 15, "Swap Y and L");
					break;
				case SWAP_XR:
					drawString(bitmap, 200, 160, 15, "Swap X and R");
					break;
				case SWAP_YR:
					drawString(bitmap, 200, 160, 15, "Swap Y and R");
					break;
				default:
					drawString(bitmap, 200, 160, 15, "Broken Config");
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
