#ifndef PHOBGCC_H
#define PHOBGCC_H

#include <algorithm>
#include <cmath>
using std::min;
using std::max;

//Uncomment to get a glowing LED on Teensy 4.
//#define ENABLE_LED

//Uncomment the appropriate #include line for your hardware by deleting the two slashes at the beginning of the line.
//#include "../teensy/Phob1_0Teensy3_2.h"          // For PhobGCC board 1.0 with Teensy 3.2
//#include "../teensy/Phob1_0Teensy3_2DiodeShort.h"// For PhobGCC board 1.0 with Teensy 3.2 and the diode shorted
//#include "../teensy/Phob1_1Teensy3_2.h"          // For PhobGCC board 1.1 with Teensy 3.2
//#include "../teensy/Phob1_1Teensy3_2DiodeShort.h"// For PhobGCC board 1.1 with Teensy 3.2 and the diode shorted
//#include "../teensy/Phob1_1Teensy4_0.h"          // For PhobGCC board 1.1 with Teensy 4.0
//#include "../teensy/Phob1_1Teensy4_0DiodeShort.h"// For PhobGCC board 1.1 with Teensy 4.0 and the diode shorted
//#include "../teensy/Phob1_2Teensy4_0.h"          // For PhobGCC board 1.2.x with Teensy 4.0
#include "../rp2040/include/PicoProtoboard.h"    // For a protoboard with a Pico on it, used for developing for the RP2040
//#include "../rp2040/include/Phob2_0.h"           // For PhobGCC Board 2.0 with RP2040

#include "structsAndEnums.h"
#include "variables.h"
#include "filter.h"
#include "stick.h"
#include "../extras/extras.h"

//#define BUILD_RELEASE
#define BUILD_DEV

//This is just an integer.
#define SW_VERSION 28

ControlConfig _controls{
	.jumpConfig = DEFAULTJUMP,
	.jumpConfigMin = DEFAULTJUMP,
	.jumpConfigMax = SWAP_YR,
	.lConfig = 0,
	.rConfig = 0,
	.triggerConfigMin = 0,
	.triggerConfigMax = 6,
	.triggerDefault = 0,
	.lTriggerOffset = 49,
	.rTriggerOffset = 49,
	.triggerMin = 49,
	.triggerMax = 227,
	.cXOffset = 0,
	.cYOffset = 0,
	.cMax = 127,
	.cMin = -127,
	.rumble = 9,
	.rumbleMin = 0,
	.rumbleMax = 11,
	.rumbleDefault = 9,//5 is the max for 3v cell rumble, 9 is for oem-feeling normal rumble motors
	.rumbleFactory = 9,
	.safeMode = true,
	.autoInit = false,
	.lTrigInitial = 0,
	.rTrigInitial = 0,
	.xSnapback = 4,
	.ySnapback = 4,
	.snapbackMin = 0,
	.snapbackMax = 10,
	.snapbackDefault = 4,
	.snapbackFactoryAX = 4,
	.snapbackFactoryAY = 4,
	.axSmoothing = 0,
	.aySmoothing = 0,
	.cxSmoothing = 0,
	.cySmoothing = 0,
	.smoothingMin = 0,
	.smoothingMax = 9,
	.snapbackFactoryCX = 0,
	.snapbackFactoryCY = 0,
	.smoothingFactoryAX = 0,
	.smoothingFactoryAY = 0,
	.axWaveshaping = 0,
	.ayWaveshaping = 0,
	.cxWaveshaping = 0,
	.cyWaveshaping = 0,
	.waveshapingMin = 0,
	.waveshapingMax = 15,
	.waveshapingFactoryAX = 0,
	.waveshapingFactoryAY = 0,
	.waveshapingFactoryCX = 0,
	.waveshapingFactoryCY = 0
};

FilterGains _gains {//these values are for 800 hz, recomputeGains converts them to what is needed for the actual frequenc
	.maxStick = 100,
	.xVelDecay = 0.1,
	.yVelDecay = 0.1,
	.xVelPosFactor = 0.01,
	.yVelPosFactor = 0.01,
	.xVelDamp = 0.125,
	.yVelDamp = 0.125,
	.velThresh = 1.00,
	.accelThresh = 3.00,
	.xSmoothing = 0.0,
	.ySmoothing = 0.0,
	.cXSmoothing = 0.0,
	.cYSmoothing = 0.0
};
FilterGains _normGains;//this gets filled by recomputeGains();

Pins _pinList {
	.pinLa = _pinLa,
	.pinRa = _pinRa,
	.pinL  = _pinL,
	.pinR  = _pinR,
	.pinAx = _pinAx,
	.pinAy = _pinAy,
	.pinCx = _pinCx,
	.pinCy = _pinCy,
	.pinRX = _pinRX,
	.pinTX = _pinTX,
	.pinDr = _pinDr,
	.pinDu = _pinDu,
	.pinDl = _pinDl,
	.pinDd = _pinDd,
	.pinX  = _pinX,
	.pinY  = _pinY,
	.pinA  = _pinA,
	.pinB  = _pinB,
	.pinZ  = _pinZ,
	.pinS  = _pinS
};

int calcRumblePower(const int rumble){
	if(rumble > 0) {
		return pow(2.0, 7+((rumble-3)/8.0)); //should be 256 when rumble is 11
	} else {
		return 0;
	}
}

float velDampFromSnapback(const int snapback){
	return 0.125 * pow(2, (snapback-4)/3.0);//4 should yield 0.125, 10 should yield 0.5, don't care about 0
}

void freezeSticks(const int time, Buttons &btn, Buttons &hardware) {
	btn.Cx = (uint8_t) (255);
	btn.Cy = (uint8_t) (255);
	btn.Ax = (uint8_t) (255);
	btn.Ay = (uint8_t) (255);
	btn.La = (uint8_t) (255 + 60.0);
	btn.Ra = (uint8_t) (255 + 60.0);

	btn.A = (uint8_t) 0;
	btn.B = (uint8_t) 0;
	btn.X = (uint8_t) 0;
	btn.Y = (uint8_t) 0;
	btn.L = (uint8_t) 0;
	btn.R = (uint8_t) 0;
	btn.Z = (uint8_t) 0;
	btn.S = (uint8_t) 0;

	hardware.A = (uint8_t) 0;
	hardware.B = (uint8_t) 0;
	hardware.X = (uint8_t) 0;
	hardware.Y = (uint8_t) 0;
	hardware.L = (uint8_t) 0;
	hardware.R = (uint8_t) 0;
	hardware.Z = (uint8_t) 0;
	hardware.S = (uint8_t) 0;

	int startTime = millis();
	int delta = 0;
	while(delta < time){
		delta = millis() - startTime;
	}
}

void freezeSticksToggleIndicator(const int time, Buttons &btn, Buttons &hardware, bool toggle) {
	btn.Cx = (uint8_t) (_floatOrigin + (toggle ? 50 : -50));
	btn.Cy = (uint8_t) (_floatOrigin + (toggle ? 50 : -50));
	btn.Ax = (uint8_t) (_floatOrigin + (toggle ? 50 : -50));
	btn.Ay = (uint8_t) (_floatOrigin + (toggle ? 50 : -50));
	btn.La = (uint8_t) (255 + 60.0);
	btn.Ra = (uint8_t) (255 + 60.0);

	btn.A = (uint8_t) 0;
	btn.B = (uint8_t) 0;
	btn.X = (uint8_t) 0;
	btn.Y = (uint8_t) 0;
	btn.L = (uint8_t) 0;
	btn.R = (uint8_t) 0;
	btn.Z = (uint8_t) 0;
	btn.S = (uint8_t) 0;

	hardware.L = (uint8_t) 0;
	hardware.R = (uint8_t) 0;
	hardware.X = (uint8_t) 0;
	hardware.Y = (uint8_t) 0;
	hardware.Z = (uint8_t) 0;

	int startTime = millis();
	int delta = 0;
	while(delta < time){
		delta = millis() - startTime;
	}
}

//This clears all the buttons but doesn't overwrite the sticks or shoulder buttons.
void clearButtons(const int time, Buttons &btn, Buttons &hardware) {
	btn.A = (uint8_t) 0;
	btn.B = (uint8_t) 0;
	btn.X = (uint8_t) 0;
	btn.Y = (uint8_t) 0;
	btn.L = (uint8_t) 0;
	btn.R = (uint8_t) 0;
	btn.Z = (uint8_t) 0;
	btn.S = (uint8_t) 0;

	hardware.A = (uint8_t) 0;
	hardware.B = (uint8_t) 0;
	hardware.X = (uint8_t) 0;
	hardware.Y = (uint8_t) 0;
	hardware.L = (uint8_t) 0;
	hardware.R = (uint8_t) 0;
	hardware.Z = (uint8_t) 0;
	hardware.S = (uint8_t) 0;

	int startTime = millis();
	int delta = 0;
	while(delta < time){
		delta = millis() - startTime;
	}
}

void showRumble(const int time, Buttons &btn, Buttons &hardware, ControlConfig &controls) {
	btn.Cx = (uint8_t) _intOrigin;
	btn.Cy = (uint8_t) (controls.rumble + _floatOrigin);
	clearButtons(time, btn, hardware);

	setRumbleSetting(controls.rumble);
#ifdef BATCHSETTINGS
	commitSettings();
#endif //BATCHSETTINGS
}

void changeRumble(const Increase increase, Buttons &btn, Buttons &hardware, ControlConfig &controls) {
#ifdef ARDUINO
	Serial.println("changing rumble");
#endif //ARDUINO
	if(increase == INCREASE) {
		controls.rumble += 1;
	} else {
		controls.rumble -= 1;
	}
	if(controls.rumble > controls.rumbleMax) {
		controls.rumble = controls.rumbleMax;
	}
	if(controls.rumble < controls.rumbleMin) {
		controls.rumble = controls.rumbleMin;
	}

	_rumblePower = calcRumblePower(controls.rumble);
	showRumble(750, btn, hardware, controls);
}

//Make it so you don't need to press B.
//This is only good if the sticks are calibrated, so
// the setting auto-resets whenever you hard reset or recalibrate.
void changeAutoInit(Buttons &btn, Buttons &hardware, ControlConfig &controls) {
	if(controls.autoInit == 0) {
		controls.autoInit = 1;
	} else {
		controls.autoInit = 0;
	}

	//move sticks up-right for on, down-left for off
	freezeSticksToggleIndicator(2000, btn, hardware, (controls.autoInit == 1));

	setAutoInitSetting(controls.autoInit);
#ifdef BATCHSETTINGS
	commitSettings();
#endif //BATCHSETTINGS
}

void adjustSnapback(const WhichAxis axis, const Increase increase, Buttons &btn, Buttons &hardware, ControlConfig &controls, FilterGains &gains, FilterGains &normGains){
#ifdef ARDUINO
	Serial.println("adjusting snapback filtering");
#endif //ARDUINO
	if(axis == XAXIS && increase == INCREASE){
		controls.xSnapback = min(controls.xSnapback+1, controls.snapbackMax);
#ifdef ARDUINO
		Serial.print("x snapback filtering increased to:");
		Serial.println(controls.xSnapback);
#endif //ARDUINO
	}
	else if(axis == XAXIS && increase == DECREASE){
		controls.xSnapback = max(controls.xSnapback-1, controls.snapbackMin);
#ifdef ARDUINO
		Serial.print("x snapback filtering decreased to:");
		Serial.println(controls.xSnapback);
#endif //ARDUINO
	}

	if(axis == YAXIS && increase == INCREASE){
		controls.ySnapback = min(controls.ySnapback+1, controls.snapbackMax);
#ifdef ARDUINO
		Serial.print("y snapback filtering increased to:");
		Serial.println(controls.ySnapback);
#endif //ARDUINO
	}
	else if(axis == YAXIS && increase == DECREASE){
		controls.ySnapback = max(controls.ySnapback-1, controls.snapbackMin);
#ifdef ARDUINO
		Serial.print("y snapback filtering decreased to:");
		Serial.println(controls.ySnapback);
#endif //ARDUINO
	}

	gains.xVelDamp = velDampFromSnapback(controls.xSnapback);
	gains.yVelDamp = velDampFromSnapback(controls.ySnapback);

    //recompute the intermediate gains used directly by the kalman filter
    recomputeGains(gains, normGains);

	btn.Cx = (uint8_t) (controls.xSnapback + _floatOrigin);
	btn.Cy = (uint8_t) (controls.ySnapback + _floatOrigin);

	clearButtons(750, btn, hardware);

	setXSnapbackSetting(controls.xSnapback);
	setYSnapbackSetting(controls.ySnapback);
#ifdef BATCHSETTINGS
	commitSettings();
#endif //BATCHSETTINGS
}

void adjustWaveshaping(const WhichStick whichStick, const WhichAxis axis, const Increase increase, Buttons &btn, Buttons &hardware, ControlConfig &controls){
#ifdef ARDUINO
	Serial.println("adjusting waveshaping");
#endif //ARDUINO
	if(whichStick == ASTICK){
		if(axis == XAXIS){
			if(increase == INCREASE){
				controls.axWaveshaping = min(controls.axWaveshaping+1, controls.waveshapingMax);
			} else {
				controls.axWaveshaping = max(controls.axWaveshaping-1, controls.waveshapingMin);
			}
		} else {//y axis
			if(increase == INCREASE){
				controls.ayWaveshaping = min(controls.ayWaveshaping+1, controls.waveshapingMax);
			} else {
				controls.ayWaveshaping = max(controls.ayWaveshaping-1, controls.waveshapingMin);
			}
		}
		setWaveshapingSetting(controls.axWaveshaping, ASTICK, XAXIS);
		setWaveshapingSetting(controls.ayWaveshaping, ASTICK, YAXIS);
		btn.Cx = (uint8_t) (controls.axWaveshaping + _floatOrigin);
		btn.Cy = (uint8_t) (controls.ayWaveshaping + _floatOrigin);
	} else {//c-stick
		if(axis == XAXIS){
			if(increase == INCREASE){
				controls.cxWaveshaping = min(controls.cxWaveshaping+1, controls.waveshapingMax);
			} else {
				controls.cxWaveshaping = max(controls.cxWaveshaping-1, controls.waveshapingMin);
			}
		} else {
			if(increase == INCREASE){
				controls.cyWaveshaping = min(controls.cyWaveshaping+1, controls.waveshapingMax);
			} else {
				controls.cyWaveshaping = max(controls.cyWaveshaping-1, controls.waveshapingMin);
			}
		}
		setWaveshapingSetting(controls.cxWaveshaping, CSTICK, XAXIS);
		setWaveshapingSetting(controls.cyWaveshaping, CSTICK, YAXIS);
		btn.Cx = (uint8_t) (controls.cxWaveshaping + _floatOrigin);
		btn.Cy = (uint8_t) (controls.cyWaveshaping + _floatOrigin);
	}
#ifdef BATCHSETTINGS
	commitSettings();
#endif //BATCHSETTINGS

	clearButtons(750, btn, hardware);
}

void adjustSmoothing(const WhichAxis axis, const Increase increase, Buttons &btn, Buttons &hardware, ControlConfig &controls, FilterGains &gains, FilterGains &normGains) {
#ifdef ARDUINO
	Serial.println("Adjusting Smoothing");
#endif //ARDUINO
	if (axis == XAXIS && increase == INCREASE) {
		controls.axSmoothing++;
		if(controls.axSmoothing > controls.smoothingMax) {
			controls.axSmoothing = controls.smoothingMax;
		}
		gains.xSmoothing = controls.axSmoothing/10.0f;
		setXSmoothingSetting(controls.axSmoothing);
#ifdef ARDUINO
		Serial.print("X Smoothing increased to:");
		Serial.println(controls.axSmoothing);
#endif //ARDUINO
	} else if(axis == XAXIS && increase == DECREASE) {
		controls.axSmoothing--;
		if(controls.axSmoothing < controls.smoothingMin) {
			controls.axSmoothing = controls.smoothingMin;
		}
		gains.xSmoothing = controls.axSmoothing/10.0f;
		setXSmoothingSetting(controls.axSmoothing);
#ifdef ARDUINO
		Serial.print("X Smoothing decreased to:");
		Serial.println(controls.axSmoothing);
#endif //ARDUINO
	} else if(axis == YAXIS && increase == INCREASE) {
		controls.aySmoothing++;
		if (controls.aySmoothing > controls.smoothingMax) {
			controls.aySmoothing = controls.smoothingMax;
		}
		gains.ySmoothing = controls.aySmoothing/10.0f;
		setYSmoothingSetting(controls.aySmoothing);
#ifdef ARDUINO
		Serial.print("Y Smoothing increased to:");
		Serial.println(controls.aySmoothing);
#endif //ARDUINO
	} else if(axis == YAXIS && increase == DECREASE) {
		controls.aySmoothing--;
		if (controls.aySmoothing < controls.smoothingMin) {
			controls.aySmoothing = controls.smoothingMin;
		}
		gains.ySmoothing = controls.aySmoothing/10.0f;
		setYSmoothingSetting(controls.aySmoothing);
#ifdef ARDUINO
		Serial.print("Y Smoothing decreased to:");
		Serial.println(controls.aySmoothing);
#endif //ARDUINO
	}
#ifdef BATCHSETTINGS
	commitSettings();
#endif //BATCHSETTINGS

	//recompute the intermediate gains used directly by the kalman filter
	recomputeGains(gains, normGains);

	btn.Cx = (uint8_t) (_floatOrigin + controls.axSmoothing);
	btn.Cy = (uint8_t) (_floatOrigin + controls.aySmoothing);

	clearButtons(750, btn, hardware);
}

void showAstickSettings(Buttons &btn, Buttons &hardware, const ControlConfig &controls, FilterGains &gains) {
	//Snapback on A-stick
	btn.Ax = (uint8_t) (controls.xSnapback + _floatOrigin);
	btn.Ay = (uint8_t) (controls.ySnapback + _floatOrigin);

	//Smoothing on C-stick
	btn.Cx = (uint8_t) (_floatOrigin + controls.axSmoothing);
	btn.Cy = (uint8_t) (_floatOrigin + controls.aySmoothing);

	//Waveshaping on triggers
	btn.La = (uint8_t) controls.axWaveshaping;
	btn.Ra = (uint8_t) controls.ayWaveshaping;

	clearButtons(2000, btn, hardware);
}

void adjustCstickSmoothing(const WhichAxis axis, const Increase increase, Buttons &btn, Buttons &hardware, ControlConfig &controls, FilterGains &gains, FilterGains &normGains) {
#ifdef ARDUINO
	Serial.println("Adjusting C-Stick Smoothing");
#endif //ARDUINO
	if (axis == XAXIS && increase == INCREASE) {
		controls.cxSmoothing++;
		if(controls.cxSmoothing > controls.smoothingMax) {
			controls.cxSmoothing = controls.smoothingMax;
		}
		gains.cXSmoothing = controls.cxSmoothing/10.0f;
		setCxSmoothingSetting(controls.cxSmoothing);
#ifdef ARDUINO
		Serial.print("C-Stick X Smoothing increased to:");
		Serial.println(controls.cxSmoothing);
#endif //ARDUINO
	} else if(axis == XAXIS && increase == DECREASE) {
		controls.cxSmoothing--;
		if(controls.cxSmoothing < controls.smoothingMin) {
			controls.cxSmoothing = controls.smoothingMin;
		}
		gains.cXSmoothing = controls.cxSmoothing/10.0f;
		setCxSmoothingSetting(controls.cxSmoothing);
#ifdef ARDUINO
		Serial.print("C-Stick X Smoothing decreased to:");
		Serial.println(controls.cxSmoothing);
#endif //ARDUINO
	} else if(axis == YAXIS && increase == INCREASE) {
		controls.cySmoothing++;
		if (controls.cySmoothing > controls.smoothingMax) {
			controls.cySmoothing = controls.smoothingMax;
		}
		gains.cYSmoothing = controls.cySmoothing/10.0f;
		setCySmoothingSetting(controls.cySmoothing);
#ifdef ARDUINO
		Serial.print("C-Stick Y Smoothing increased to:");
		Serial.println(controls.cySmoothing);
#endif //ARDUINO
	} else if(axis == YAXIS && increase == DECREASE) {
		controls.cySmoothing--;
		if (controls.cySmoothing < controls.smoothingMin) {
			controls.cySmoothing = controls.smoothingMin;
		}
		gains.cYSmoothing = controls.cySmoothing/10.0f;
		setCySmoothingSetting(controls.cySmoothing);
#ifdef ARDUINO
		Serial.print("C-Stick Y Smoothing decreased to:");
		Serial.println(controls.cySmoothing);
#endif //ARDUINO
	}
#ifdef BATCHSETTINGS
	commitSettings();
#endif //BATCHSETTINGS

	//recompute the intermediate gains used directly by the kalman filter
	recomputeGains(gains, normGains);

	btn.Cx = (uint8_t) (_floatOrigin + controls.cxSmoothing);
	btn.Cy = (uint8_t) (_floatOrigin + controls.cySmoothing);

	clearButtons(750, btn, hardware);
}

void adjustCstickOffset(const WhichAxis axis, const Increase increase, Buttons &btn, Buttons &hardware, ControlConfig &controls) {
#ifdef ARDUINO
	Serial.println("Adjusting C-stick Offset");
#endif //ARDUINO
	if(axis == XAXIS && increase == INCREASE) {
		controls.cXOffset++;
		if(controls.cXOffset > controls.cMax) {
			controls.cXOffset = controls.cMax;
		}
		setCxOffsetSetting(controls.cXOffset);
#ifdef ARDUINO
		Serial.print("X offset increased to:");
		Serial.println(controls.cXOffset);
#endif //ARDUINO
	} else if(axis == XAXIS && increase == DECREASE) {
		controls.cXOffset--;
		if(controls.cXOffset < controls.cMin) {
			controls.cXOffset = controls.cMin;
		}
		setCxOffsetSetting(controls.cXOffset);
#ifdef ARDUINO
		Serial.print("X offset decreased to:");
		Serial.println(controls.cXOffset);
#endif //ARDUINO
	} else if(axis == YAXIS && increase == INCREASE) {
		controls.cYOffset++;
		if(controls.cYOffset > controls.cMax) {
			controls.cYOffset = controls.cMax;
		}
		setCyOffsetSetting(controls.cYOffset);
#ifdef ARDUINO
		Serial.print("Y offset increased to:");
		Serial.println(controls.cYOffset);
#endif //ARDUINO
	} else if(axis == YAXIS && increase == DECREASE) {
		controls.cYOffset--;
		if(controls.cYOffset < controls.cMin) {
			controls.cYOffset = controls.cMin;
		}
		setCyOffsetSetting(controls.cYOffset);
#ifdef ARDUINO
		Serial.print("Y offset decreased to:");
		Serial.println(controls.cYOffset);
#endif //ARDUINO
	}
#ifdef BATCHSETTINGS
	commitSettings();
#endif //BATCHSETTINGS

	btn.Cx = (uint8_t) (_floatOrigin + controls.cXOffset);
	btn.Cy = (uint8_t) (_floatOrigin + controls.cYOffset);

	clearButtons(750, btn, hardware);
}

void showCstickSettings(Buttons &btn, Buttons &hardware, ControlConfig &controls, FilterGains &gains) {
	//Snapback/smoothing on A-stick
	btn.Ax = (uint8_t) (_floatOrigin + controls.cxSmoothing);
	btn.Ay = (uint8_t) (_floatOrigin + controls.cySmoothing);

	//Smoothing on C-stick
	btn.Cx = (uint8_t) (_floatOrigin + controls.cXOffset);
	btn.Cy = (uint8_t) (_floatOrigin + controls.cYOffset);

	//Waveshaping on triggers
	btn.La = (uint8_t) controls.cxWaveshaping;
	btn.Ra = (uint8_t) controls.cyWaveshaping;

	clearButtons(2000, btn, hardware);
}

void adjustTriggerOffset(const WhichTrigger trigger, const Increase increase, Buttons &btn, Buttons &hardware, ControlConfig &controls) {
	if(trigger == LTRIGGER && increase == INCREASE) {
		controls.lTriggerOffset++;
		if(controls.lTriggerOffset > controls.triggerMax) {
			controls.lTriggerOffset = controls.triggerMax;
		}
	} else if(trigger == LTRIGGER && increase == DECREASE) {
		controls.lTriggerOffset--;
		if(controls.lTriggerOffset < controls.triggerMin) {
			controls.lTriggerOffset = controls.triggerMin;
		}
	} else if(trigger == RTRIGGER && increase == INCREASE) {
		controls.rTriggerOffset++;
		if(controls.rTriggerOffset > controls.triggerMax) {
			controls.rTriggerOffset = controls.triggerMax;
		}
	} else if(trigger == RTRIGGER && increase == DECREASE) {
		controls.rTriggerOffset--;
		if(controls.rTriggerOffset < controls.triggerMin) {
			controls.rTriggerOffset = controls.triggerMin;
		}
	}

	setLOffsetSetting(controls.lTriggerOffset);
	setROffsetSetting(controls.rTriggerOffset);
#ifdef BATCHSETTINGS
	commitSettings();
#endif //BATCHSETTINGS

	btn.La = (uint8_t) controls.lTriggerOffset;
	btn.Ra = (uint8_t) controls.rTriggerOffset;

	if(controls.lTriggerOffset > 99) {
		btn.Ax = (uint8_t) (_floatOrigin + 100);
		btn.Cx = (uint8_t) (_floatOrigin + controls.lTriggerOffset-100);
	} else {
		btn.Cx = (uint8_t) (_floatOrigin + controls.lTriggerOffset);
	}
	if(controls.rTriggerOffset > 99) {
		btn.Ay = (uint8_t) (_floatOrigin + 100);
		btn.Cy = (uint8_t) (_floatOrigin + controls.rTriggerOffset-100);
	} else {
		btn.Cy = (uint8_t) (_floatOrigin + controls.rTriggerOffset);
	}

	clearButtons(100, btn, hardware);
}

//apply digital button swaps for L, R, or Z jumping
void applyJump(const ControlConfig &controls, const Buttons &hardware, Buttons &btn){
	switch(controls.jumpConfig){
		case SWAP_XZ:
			btn.X = hardware.Z;
			btn.Z = hardware.X;
			break;
		case SWAP_YZ:
			btn.Y = hardware.Z;
			btn.Z = hardware.Y;
			break;
		case SWAP_XL:
			btn.X = hardware.L;
			btn.L = hardware.X;
			break;
		case SWAP_YL:
			btn.Y = hardware.L;
			btn.L = hardware.Y;
			break;
		case SWAP_XR:
			btn.X = hardware.R;
			btn.R = hardware.X;
			break;
		case SWAP_YR:
			btn.Y = hardware.R;
			btn.R = hardware.Y;
			break;
		default:
			break;
			//nothing
	}
}

void setJumpConfig(JumpConfig jumpConfig, ControlConfig &controls){
#ifdef ARDUINO
	Serial.print("setting jump to: ");
#endif //ARDUINO
	if (controls.jumpConfig == jumpConfig) {
		controls.jumpConfig = DEFAULTJUMP;
#ifdef ARDUINO
		Serial.println("normal again");
#endif //ARDUINO
	} else {
		controls.jumpConfig = jumpConfig;
#ifdef ARDUINO
		switch (jumpConfig) {
			case SWAP_XZ:
				Serial.println("X<->Z");
				break;
			case SWAP_YZ:
				Serial.println("Y<->Z");
				break;
			case SWAP_XL:
				Serial.println("X<->L");
				break;
			case SWAP_YL:
				Serial.println("Y<->L");
				break;
			case SWAP_XR:
				Serial.println("X<->R");
				break;
			case SWAP_YR:
				Serial.println("Y<->R");
				break;
			default:
				Serial.println("normal");
		}
#endif //ARDUINO
	}
	setJumpSetting(controls.jumpConfig);
#ifdef BATCHSETTINGS
	commitSettings();
#endif //BATCHSETTINGS
}

void toggleExtra(ExtrasSlot slot, Buttons &btn, Buttons &hardware, ControlConfig &controls){
	ExtrasToggleFn toggleFn = extrasFunctions[slot].toggleFn;
	if (toggleFn) {
		bool toggle = toggleFn(controls.extras[slot].config);
		freezeSticksToggleIndicator(2000, btn, hardware, toggle);
	}
}

void configExtra(ExtrasSlot slot, Buttons &btn, Buttons &hardware, ControlConfig &controls){
	ExtrasConfigFn configFn = extrasFunctions[slot].configFn;
	if (configFn) {
		Cardinals dpad;
		dpad.l = btn.Dl;
		dpad.r = btn.Dr;
		dpad.u = btn.Du;
		dpad.d = btn.Dd;
		configFn(controls.extras[slot].config, dpad);
	}
}

bool checkAdjustExtra(ExtrasSlot slot, Buttons &btn, bool checkConfig){
	//Extras Toggles: Both control sticks as Up, Down, Left, or Right, and with A + B
	if (!checkConfig){
		switch(slot){
			case EXTRAS_UP:
				return (btn.Ay > (_intOrigin+48) && btn.Cy > (_intOrigin+48)
				        && btn.Ax < (_intOrigin+30) && btn.Ax > (_intOrigin-30)
				        && btn.Cx < (_intOrigin+30) && btn.Cx > (_intOrigin-30)
				        && btn.A && btn.B);
			case EXTRAS_DOWN:
				return (btn.Ay < (_intOrigin-48) && btn.Cy < (_intOrigin-48)
				        && btn.Ax < (_intOrigin+30) && btn.Ax > (_intOrigin-30)
				        && btn.Cx < (_intOrigin+30) && btn.Cx > (_intOrigin-30)
				        && btn.A && btn.B);
			case EXTRAS_LEFT:
				return (btn.Ax < (_intOrigin-48) && btn.Cx < (_intOrigin-48)
				        && btn.Ay < (_intOrigin+30) && btn.Ay > (_intOrigin-30)
				        && btn.Cy < (_intOrigin+30) && btn.Cy > (_intOrigin-30)
				        && btn.A && btn.B);
			case EXTRAS_RIGHT:
				return (btn.Ax > (_intOrigin+48) && btn.Cx > (_intOrigin+48)
				        && btn.Ay < (_intOrigin+30) && btn.Ay > (_intOrigin-30)
				        && btn.Cy < (_intOrigin+30) && btn.Cy > (_intOrigin-30)
				        && btn.A && btn.B);
			default:
				return false;
		}
	} else {
		switch(slot){
			case EXTRAS_UP:
				return (btn.Ay > (_intOrigin+48) && btn.Cy > (_intOrigin+48)
				        && btn.Ax < (_intOrigin+30) && btn.Ax > (_intOrigin-30)
				        && btn.Cx < (_intOrigin+30) && btn.Cx > (_intOrigin-30)
				        && btn.A && (btn.Du||btn.Dd||btn.Dl||btn.Dr));
			case EXTRAS_DOWN:
				return (btn.Ay < (_intOrigin-48) && btn.Cy < (_intOrigin-48)
				        && btn.Ax < (_intOrigin+30) && btn.Ax > (_intOrigin-30)
				        && btn.Cx < (_intOrigin+30) && btn.Cx > (_intOrigin-30)
				        && btn.A && (btn.Du||btn.Dd||btn.Dl||btn.Dr));
			case EXTRAS_LEFT:
				return (btn.Ax < (_intOrigin-48) && btn.Cx < (_intOrigin-48)
				        && btn.Ay < (_intOrigin+30) && btn.Ay > (_intOrigin-30)
				        && btn.Cy < (_intOrigin+30) && btn.Cy > (_intOrigin-30)
				        && btn.A && (btn.Du||btn.Dd||btn.Dl||btn.Dr));
			case EXTRAS_RIGHT:
				return (btn.Ax > (_intOrigin+48) && btn.Cx > (_intOrigin+48)
				        && btn.Ay < (_intOrigin+30) && btn.Ay > (_intOrigin-30)
				        && btn.Cy < (_intOrigin+30) && btn.Cy > (_intOrigin-30)
				        && btn.A && (btn.Du||btn.Dd||btn.Dl||btn.Dr));
			default:
				return false;
		}
	}
	return false;
}

void nextTriggerState(WhichTrigger trigger, Buttons &btn, Buttons &hardware, ControlConfig &controls) {
	if(trigger == LTRIGGER) {
		if(controls.lConfig >= controls.triggerConfigMax) {
			controls.lConfig = 0;
		} else {
			controls.lConfig = controls.lConfig + 1;
		}
	} else {
		if(controls.rConfig >= controls.triggerConfigMax) {
			controls.rConfig = 0;
		} else {
			controls.rConfig = controls.rConfig + 1;
		}
	}
	setLSetting(controls.lConfig);
	setRSetting(controls.rConfig);
#ifdef BATCHSETTINGS
	commitSettings();
#endif //BATCHSETTINGS

	//if the modes are incompatible due to mode 5, make it show -100 on the stick that isn't mode 5
	//(user-facing mode 5)
	int lConfig = controls.lConfig;
	int rConfig = controls.rConfig;
	int triggerConflict = 0;
	if(rConfig == 4 && (lConfig == 0 || lConfig == 2 || lConfig == 3)) {
		triggerConflict = -100;
	}
	if(lConfig == 4 && (rConfig == 0 || rConfig == 2 || rConfig == 3)) {
		triggerConflict = -100;
	}
	//We want to one-index the modes for the users, so we add 1 here
	btn.Ay = (uint8_t) (_floatOrigin + triggerConflict);
	btn.Ax = (uint8_t) (_floatOrigin + controls.lConfig + 1);
	btn.Cy = (uint8_t) (_floatOrigin + triggerConflict);
	btn.Cx = (uint8_t) (_floatOrigin + controls.rConfig + 1);

	clearButtons(1000, btn, hardware);
}

void initializeButtons(const Pins &pin, Buttons &btn,int &startUpLa, int &startUpRa){
	//set the analog stick values to the chosen center value that will be reported to the console on startup
	//We choose 127 (_intOrigin) for this, and elsewhere we use an offset of 127.5 (_floatOrigin) truncated to int in order to round properly
	btn.Ax = _intOrigin;
	btn.Ay = _intOrigin;
	btn.Cx = _intOrigin;
	btn.Cy = _intOrigin;

	//read the ADC inputs for the analog triggers a few times and choose the startup value to be the maximum that was recorded
	//these values could be used as offsets to set particular trigger values
	startUpLa = 0;
	startUpRa = 0;
	for(int i = 0; i <64; i++){
		startUpLa = max(startUpLa,readLa(pin, 0, 1));
		startUpRa = max(startUpRa,readRa(pin, 0, 1));
	}
	//set the trigger values to this measured startup value
	btn.La = 0;
	btn.Ra = 0;

}

//Take tempCalPoints and use it to generate new stick cal parameters to be used
void applyCalFromPoints(const WhichStick whichStick, float notchAngles[], const float tempCalPointsX[], const float tempCalPointsY[], NotchStatus notchStatus[], float measuredNotchAngles[], StickParams &stickParams) {
	//recall previous notch angles
	getNotchAnglesSetting(notchAngles, whichStick);
	//make temp temp cal points that are missing all tertiary notches so that we get a neutral grid
	float temptempCalPointsX[_noOfCalibrationPoints];
	float temptempCalPointsY[_noOfCalibrationPoints];
	stripCalPoints(tempCalPointsX, tempCalPointsY, temptempCalPointsX, temptempCalPointsY);
	//clean the stripped calibration points, use default angles
	float cleanedPointsX[_noOfNotches+1];
	float cleanedPointsY[_noOfNotches+1];
	float notchPointsX[_noOfNotches+1];
	float notchPointsY[_noOfNotches+1];
	cleanCalPoints(temptempCalPointsX, temptempCalPointsY, _notchAngleDefaults, cleanedPointsX, cleanedPointsY, notchPointsX, notchPointsY, notchStatus);
	linearizeCal(cleanedPointsX, cleanedPointsY, cleanedPointsX, cleanedPointsY, stickParams);
	notchCalibrate(cleanedPointsX, cleanedPointsY, notchPointsX, notchPointsY, _noOfNotches, stickParams);
	//apply the calibration to the original measured values including any tertiaries; we don't care about the angles
	cleanCalPoints(tempCalPointsX, tempCalPointsY, _notchAngleDefaults, cleanedPointsX, cleanedPointsY, notchPointsX, notchPointsY, notchStatus);
	float transformedX[_noOfNotches+1];
	float transformedY[_noOfNotches+1];
	transformCalPoints(cleanedPointsX, cleanedPointsY, transformedX, transformedY, stickParams);
	//compute the angles for those notches into measuredNotchAngles, using the default angles for the diagonals
	computeStickAngles(transformedX, transformedY, measuredNotchAngles);
	//clean full cal points again, feeding those angles in
	cleanCalPoints(tempCalPointsX, tempCalPointsY, measuredNotchAngles, cleanedPointsX, cleanedPointsY, notchPointsX, notchPointsY, notchStatus);
	//clear unused notch angles
	cleanNotches(notchAngles, measuredNotchAngles, notchStatus);
	//clean full cal points again again, feeding those measured angles in for missing tertiary notches
	cleanCalPoints(tempCalPointsX, tempCalPointsY, notchAngles, cleanedPointsX, cleanedPointsY, notchPointsX, notchPointsY, notchStatus);
	//linearize again
	linearizeCal(cleanedPointsX, cleanedPointsY, cleanedPointsX, cleanedPointsY, stickParams);
	//notchCalibrate again
	notchCalibrate(cleanedPointsX, cleanedPointsY, notchPointsX, notchPointsY, _noOfNotches, stickParams);
};


int readEEPROM(ControlConfig &controls, FilterGains &gains, FilterGains &normGains, StickParams &aStickParams, StickParams &cStickParams, const bool noLock = false){
	int numberOfNaN = 0;

	//get the jump setting
	controls.jumpConfig = getJumpSetting();
	if(controls.jumpConfig < controls.jumpConfigMin){
		controls.jumpConfig = DEFAULTJUMP;
		numberOfNaN++;
	}
	if(controls.jumpConfig > controls.jumpConfigMax){
		controls.jumpConfig = DEFAULTJUMP;
		numberOfNaN++;
	}

	//get the L setting
	controls.lConfig = getLSetting();
	if(controls.lConfig < controls.triggerConfigMin) {
		controls.lConfig = controls.triggerDefault;
		numberOfNaN++;
	}
	if(controls.lConfig > controls.triggerConfigMax) {
		controls.lConfig = controls.triggerDefault;
		numberOfNaN++;
	}

	//get the R setting
	controls.rConfig = getRSetting();
	if(controls.rConfig < controls.triggerConfigMin) {
		controls.rConfig = controls.triggerDefault;
		numberOfNaN++;
	}
	if(controls.rConfig > controls.triggerConfigMax) {
		controls.rConfig = controls.triggerDefault;
		numberOfNaN++;
	}

	//get the L-trigger Offset value
	controls.lTriggerOffset = getLOffsetSetting();
	if(controls.lTriggerOffset > controls.triggerMax) {
		controls.lTriggerOffset = controls.triggerMax;
		numberOfNaN++;
	} else if(controls.lTriggerOffset < controls.triggerMin) {
		controls.lTriggerOffset = controls.triggerMin;
		numberOfNaN++;
	}

	//get the R-trigger Offset value
	controls.rTriggerOffset = getROffsetSetting();
	if(controls.rTriggerOffset > controls.triggerMax) {
		controls.rTriggerOffset = controls.triggerMax;
		numberOfNaN++;
	} else if(controls.rTriggerOffset < controls.triggerMin) {
		controls.rTriggerOffset = controls.triggerMin;
		numberOfNaN++;
	}


	//get the C-stick X offset
	controls.cXOffset = getCxOffsetSetting();
	if(controls.cXOffset > controls.cMax) {
		controls.cXOffset = 0;
		numberOfNaN++;
	} else if(controls.cXOffset < controls.cMin) {
		controls.cXOffset = 0;
		numberOfNaN++;
	}

	//get the C-stick Y offset
	controls.cYOffset = getCyOffsetSetting();
	if(controls.cYOffset > controls.cMax) {
		controls.cYOffset = 0;
		numberOfNaN++;
	} else if(controls.cYOffset < controls.cMin) {
		controls.cYOffset = 0;
		numberOfNaN++;
	}

	//get the x-axis snapback correction
	controls.xSnapback = getXSnapbackSetting();
#ifdef ARDUINO
	Serial.print("the xSnapback value from eeprom is:");
	Serial.println(controls.xSnapback);
#endif //ARDUINO
	if(controls.xSnapback < controls.snapbackMin) {
		controls.xSnapback = controls.snapbackDefault;
		numberOfNaN++;
	} else if (controls.xSnapback > controls.snapbackMax) {
		controls.xSnapback = controls.snapbackDefault;
		numberOfNaN++;
	}
	gains.xVelDamp = velDampFromSnapback(controls.xSnapback);
#ifdef ARDUINO
	Serial.print("the xVelDamp value from eeprom is:");
	Serial.println(gains.xVelDamp);
#endif //ARDUINO

	//get the y-ayis snapback correction
	controls.ySnapback = getYSnapbackSetting();
#ifdef ARDUINO
	Serial.print("the ySnapback value from eeprom is:");
	Serial.println(controls.ySnapback);
#endif //ARDUINO
	if(controls.ySnapback < controls.snapbackMin) {
		controls.ySnapback = controls.snapbackDefault;
		numberOfNaN++;
	} else if (controls.ySnapback > controls.snapbackMax) {
		controls.ySnapback = controls.snapbackDefault;
		numberOfNaN++;
	}
	gains.yVelDamp = velDampFromSnapback(controls.ySnapback);
#ifdef ARDUINO
	Serial.print("the yVelDamp value from eeprom is:");
	Serial.println(gains.yVelDamp);
#endif //ARDUINO

	//get the x-axis smoothing value
	controls.axSmoothing = getXSmoothingSetting();
#ifdef ARDUINO
	Serial.print("the xSmoothing value from eeprom is:");
	Serial.println(controls.axSmoothing);
#endif //ARDUINO
	if(controls.axSmoothing > controls.smoothingMax) {
		controls.axSmoothing = controls.smoothingMin;
		numberOfNaN++;
#ifdef ARDUINO
		Serial.print("the xSmoothing value was adjusted to:");
		Serial.println(controls.axSmoothing);
#endif //ARDUINO
	} else if(controls.axSmoothing < controls.smoothingMin) {
		controls.axSmoothing = controls.smoothingMin;
		numberOfNaN++;
#ifdef ARDUINO
		Serial.print("the xSmoothing value was adjusted to:");
		Serial.println(controls.axSmoothing);
#endif //ARDUINO
	}
	gains.xSmoothing = controls.axSmoothing / 10.0f;

	//get the y-axis smoothing value
	controls.aySmoothing = getYSmoothingSetting();
#ifdef ARDUINO
	Serial.print("the ySmoothing value from eeprom is:");
	Serial.println(controls.aySmoothing);
#endif //ARDUINO
	if(controls.aySmoothing > controls.smoothingMax) {
		controls.aySmoothing = controls.smoothingMin;
		numberOfNaN++;
#ifdef ARDUINO
		Serial.print("the ySmoothing value was adjusted to:");
		Serial.println(controls.aySmoothing);
#endif //ARDUINO
	} else if(controls.aySmoothing < controls.smoothingMin) {
		controls.aySmoothing = controls.smoothingMin;
		numberOfNaN++;
#ifdef ARDUINO
		Serial.print("the ySmoothing value was adjusted to:");
		Serial.println(controls.aySmoothing);
#endif //ARDUINO
	}
	gains.ySmoothing = controls.aySmoothing / 10.0f;

	//get the c-stick x-axis smoothing value
	controls.cxSmoothing = getCxSmoothingSetting();
#ifdef ARDUINO
	Serial.print("the cXSmoothing value from eeprom is:");
	Serial.println(controls.cxSmoothing);
#endif //ARDUINO
	if(controls.cxSmoothing > controls.smoothingMax) {
		controls.cxSmoothing = controls.smoothingMin;
		numberOfNaN++;
#ifdef ARDUINO
		Serial.print("the cXSmoothing value was adjusted to:");
		Serial.println(controls.cxSmoothing);
#endif //ARDUINO
	} else if(controls.cxSmoothing < controls.smoothingMin) {
		controls.cxSmoothing = controls.smoothingMin;
		numberOfNaN++;
#ifdef ARDUINO
		Serial.print("the cXSmoothing value was adjusted to:");
		Serial.println(controls.cxSmoothing);
#endif //ARDUINO
	}
	gains.cXSmoothing = controls.cxSmoothing / 10.0f;

	//get the c-stick y-axis smoothing value
	controls.cySmoothing = getCySmoothingSetting();
#ifdef ARDUINO
	Serial.print("the cYSmoothing value from eeprom is:");
	Serial.println(controls.cySmoothing);
#endif //ARDUINO
	if(controls.cySmoothing > controls.smoothingMax) {
		controls.cySmoothing = controls.smoothingMin;
		numberOfNaN++;
#ifdef ARDUINO
		Serial.print("the cYSmoothing value was adjusted to:");
		Serial.println(controls.cySmoothing);
#endif //ARDUINO
	} else if(controls.cySmoothing < controls.smoothingMin) {
		controls.cySmoothing = controls.smoothingMin;
		numberOfNaN++;
#ifdef ARDUINO
		Serial.print("the cYSmoothing value was adjusted to:");
		Serial.println(controls.cySmoothing);
#endif //ARDUINO
	}
	gains.cYSmoothing = controls.cySmoothing/10.0f;

	//get the a-stick x-axis waveshaping value
	controls.axWaveshaping = getWaveshapingSetting(ASTICK, XAXIS);
#ifdef ARDUINO
	Serial.print("the axWaveshaping value from eeprom is:");
	Serial.println(controls.axWaveshaping);
#endif //ARDUINO
	if(controls.axWaveshaping < controls.waveshapingMin) {
		controls.axWaveshaping = controls.waveshapingMin;
		numberOfNaN++;
	} else if (controls.axWaveshaping > controls.waveshapingMax) {
		controls.axWaveshaping = controls.waveshapingMin;
		numberOfNaN++;
	}

	//get the a-stick y-axis waveshaping value
	controls.ayWaveshaping = getWaveshapingSetting(ASTICK, YAXIS);
#ifdef ARDUINO
	Serial.print("the ayWaveshaping value from eeprom is:");
	Serial.println(controls.ayWaveshaping);
#endif //ARDUINO
	if(controls.ayWaveshaping < controls.waveshapingMin) {
		controls.ayWaveshaping = controls.waveshapingMin;
		numberOfNaN++;
	} else if (controls.ayWaveshaping > controls.waveshapingMax) {
		controls.ayWaveshaping = controls.waveshapingMax;
		numberOfNaN++;
	}

	//get the c-stick x-axis waveshaping value
	controls.cxWaveshaping = getWaveshapingSetting(CSTICK, XAXIS);
#ifdef ARDUINO
	Serial.print("the cxWaveshaping value from eeprom is:");
	Serial.println(controls.cxWaveshaping);
#endif //ARDUINO
	if(controls.cxWaveshaping < controls.waveshapingMin) {
		controls.cxWaveshaping = controls.waveshapingMin;
		numberOfNaN++;
	} else if (controls.cxWaveshaping > controls.waveshapingMax) {
		controls.cxWaveshaping = controls.waveshapingMin;
		numberOfNaN++;
	}

	//get the c-stick y-axis waveshaping value
	controls.cyWaveshaping = getWaveshapingSetting(CSTICK, YAXIS);
#ifdef ARDUINO
	Serial.print("the cyWaveshaping value from eeprom is:");
	Serial.println(controls.cyWaveshaping);
#endif //ARDUINO
	if(controls.cyWaveshaping < controls.waveshapingMin) {
		controls.cyWaveshaping = controls.waveshapingMin;
		numberOfNaN++;
	} else if (controls.cyWaveshaping > controls.waveshapingMax) {
		controls.cyWaveshaping = controls.waveshapingMin;
		numberOfNaN++;
	}

	if(controls.axWaveshaping != 0){
#ifdef ARDUINO
		Serial.print("the axWaveshaping coefficient is: ");
		Serial.println(1/calcWaveshapeMult(controls.axWaveshaping));
#endif //ARDUINO
	}

	//recompute the intermediate gains used directly by the kalman filter
	recomputeGains(gains, normGains);

	//Get the rumble value
	controls.rumble = getRumbleSetting();
#ifdef ARDUINO
	Serial.print("Rumble value before fixing: ");
	Serial.println(controls.rumble);
#endif //ARDUINO
	if(std::isnan(controls.rumble)) {
		controls.rumble = controls.rumbleDefault;
		numberOfNaN++;
	}
	if(controls.rumble < controls.rumbleMin) {
		controls.rumble = controls.rumbleDefault;
	}
	if(controls.rumble > controls.rumbleMax) {
		controls.rumble = controls.rumbleDefault;
	}
	_rumblePower = calcRumblePower(controls.rumble);
#ifdef ARDUINO
	Serial.print("Rumble value: ");
	Serial.println(controls.rumble);
	Serial.print("Rumble power: ");
	Serial.println(_rumblePower);
#endif //ARDUINO

	//Get the autoinit value
	controls.autoInit = getAutoInitSetting();
	if(controls.autoInit < 0) {
		controls.autoInit = 0;
		numberOfNaN++;
	}
	if(controls.autoInit > 1) {
		controls.autoInit = 0;
		numberOfNaN++;
	}
#ifdef ARDUINO
	Serial.print("Auto init: ");
	Serial.println(controls.autoInit);
#endif //ARDUINO

	//get the calibration points collected during the last A stick calibration
	float tempCalPointsX[_noOfCalibrationPoints];
	float tempCalPointsY[_noOfCalibrationPoints];
	float notchAngles[_noOfNotches];
	getPointsSetting(tempCalPointsX, ASTICK, XAXIS);
	getPointsSetting(tempCalPointsY, ASTICK, YAXIS);
	getNotchAnglesSetting(notchAngles, ASTICK);

	float cleanedPointsX[_noOfNotches+1];
	float cleanedPointsY[_noOfNotches+1];
	float notchPointsX[_noOfNotches+1];
	float notchPointsY[_noOfNotches+1];
	NotchStatus notchStatus[_noOfNotches];

	cleanCalPoints(tempCalPointsX, tempCalPointsY, notchAngles, cleanedPointsX, cleanedPointsY, notchPointsX, notchPointsY, notchStatus);
#ifdef ARDUINO
	Serial.println("calibration points cleaned");
#endif //ARDUINO
	linearizeCal(cleanedPointsX, cleanedPointsY, cleanedPointsX, cleanedPointsY, aStickParams);
#ifdef ARDUINO
	Serial.println("A stick linearized");
#endif //ARDUINO
	notchCalibrate(cleanedPointsX, cleanedPointsY, notchPointsX, notchPointsY, _noOfNotches, aStickParams);

	//get the calibration points collected during the last A stick calibration
	getPointsSetting(tempCalPointsX, CSTICK, XAXIS);
	getPointsSetting(tempCalPointsY, CSTICK, YAXIS);
	getNotchAnglesSetting(notchAngles, CSTICK);
	cleanCalPoints(tempCalPointsX, tempCalPointsY, notchAngles, cleanedPointsX, cleanedPointsY, notchPointsX, notchPointsY, notchStatus);
#ifdef ARDUINO
	Serial.println("calibration points cleaned");
#endif //ARDUINO
	linearizeCal(cleanedPointsX, cleanedPointsY, cleanedPointsX, cleanedPointsY, cStickParams);
#ifdef ARDUINO
	Serial.println("C stick linearized");
#endif //ARDUINO
	notchCalibrate(cleanedPointsX, cleanedPointsY, notchPointsX, notchPointsY, _noOfNotches, cStickParams);

	//read in extras settings
	for(int extra=0; extra<EXTRAS_SIZE; extra++){
		ExtrasSlot slot = (ExtrasSlot)extra;
		for(int offset=0; offset<4; offset++){
			controls.extras[slot].config[offset].intValue = getExtrasSettingInt(slot, offset);
		}
	}

	//Migration
	const int schema = getSchemaSetting();
#ifdef ARDUINO
	Serial.print("Saved settings schema: ");
	Serial.println(schema);
#endif //ARDUINO
	bool migrating = false;
	switch(schema) {
		case -1:
			migrating = true;
#ifdef ARDUINO
			Serial.println("Updating settings from unitialized");
#endif //ARDUINO
			controls.rumble = controls.rumble!=0 ? controls.rumble + 4 : 0;
			_rumblePower = calcRumblePower(controls.rumble);
			setRumbleSetting(controls.rumble);
#ifdef ARDUINO
			Serial.print("Rumble value changed to: ");
			Serial.println(controls.rumble);
			Serial.print("Rumble power now: ");
			Serial.println(_rumblePower);
#endif //ARDUINO
			//I'd like it to change smoothing, but it's way too complicated
		case 28:
			//migrating = true;//uncomment when we do have it migrate
#ifdef ARDUINO
			Serial.println("Schema is now current");
#endif //ARDUINO
			//fallthrough
		default:
			if(migrating) {
#ifdef ARDUINO
				Serial.println("Updating saved settings schema");
#endif //ARDUINO
				setSchemaSetting(SW_VERSION);
#ifdef BATCHSETTINGS
				commitSettings(noLock);
#endif //BATCHSETTINGS
			}
	}
	return numberOfNaN;
}

void resetDefaults(HardReset reset, ControlConfig &controls, FilterGains &gains, FilterGains &normGains, StickParams &aStickParams, StickParams &cStickParams, const bool noLock = false){
#ifdef ARDUINO
	Serial.println("RESETTING ALL DEFAULTS");
#endif //ARDUINO

	controls.jumpConfig = DEFAULTJUMP;
	setJumpSetting(controls.jumpConfig);

	controls.lConfig = controls.triggerDefault;
	controls.rConfig = controls.triggerDefault;
	setLSetting(controls.lConfig);
	setRSetting(controls.rConfig);

	controls.cXOffset = 0;
	controls.cYOffset = 0;
	setCxOffsetSetting(controls.cXOffset);
	setCyOffsetSetting(controls.cYOffset);

	if(reset == FACTORY){
		controls.xSnapback = controls.snapbackFactoryAX;
		controls.ySnapback = controls.snapbackFactoryAY;
	} else {
		controls.xSnapback = controls.snapbackDefault;
		controls.ySnapback = controls.snapbackDefault;
	}
	setXSnapbackSetting(controls.xSnapback);
	gains.xVelDamp = velDampFromSnapback(controls.xSnapback);
	setYSnapbackSetting(controls.ySnapback);
	gains.yVelDamp = velDampFromSnapback(controls.ySnapback);

	if(reset == FACTORY){
		controls.axSmoothing = controls.smoothingFactoryAX;
		controls.aySmoothing = controls.smoothingFactoryAY;
		controls.cxSmoothing = controls.snapbackFactoryCX;
		controls.cySmoothing = controls.snapbackFactoryCY;
	} else {
		controls.axSmoothing = controls.smoothingMin;
		controls.aySmoothing = controls.smoothingMin;
		controls.cxSmoothing = controls.smoothingMin;
		controls.cySmoothing = controls.smoothingMin;
	}
	gains.xSmoothing  = controls.axSmoothing / 10.0f;
	gains.ySmoothing  = controls.aySmoothing / 10.0f;
	gains.cXSmoothing = controls.cxSmoothing / 10.0f;
	gains.cYSmoothing = controls.cySmoothing / 10.0f;
	setXSmoothingSetting(controls.axSmoothing);
	setYSmoothingSetting(controls.aySmoothing);
	setCxSmoothingSetting(controls.cxSmoothing);
	setCySmoothingSetting(controls.cySmoothing);
	//recompute the intermediate gains used directly by the kalman filter
	recomputeGains(gains, normGains);

	if(reset == FACTORY){
		controls.axWaveshaping = controls.waveshapingFactoryAX;
		controls.ayWaveshaping = controls.waveshapingFactoryAY;
		controls.cxWaveshaping = controls.waveshapingFactoryCX;
		controls.cyWaveshaping = controls.waveshapingFactoryCY;
	} else {
		controls.axWaveshaping = controls.waveshapingMin;
		controls.ayWaveshaping = controls.waveshapingMin;
		controls.cxWaveshaping = controls.waveshapingMin;
		controls.cyWaveshaping = controls.waveshapingMin;
	}
	setWaveshapingSetting(controls.waveshapingMin, ASTICK, XAXIS);
	setWaveshapingSetting(controls.waveshapingMin, ASTICK, YAXIS);
	setWaveshapingSetting(controls.waveshapingMin, CSTICK, XAXIS);
	setWaveshapingSetting(controls.waveshapingMin, CSTICK, YAXIS);

	controls.lTriggerOffset = controls.triggerMin;
	controls.rTriggerOffset = controls.triggerMin;
	setLOffsetSetting(controls.lTriggerOffset);
	setROffsetSetting(controls.rTriggerOffset);

	if(reset == FACTORY){
		controls.rumble = controls.rumbleFactory;
	} else {
		controls.rumble = controls.rumbleDefault;
	}
	_rumblePower = calcRumblePower(controls.rumble);
	setRumbleSetting(controls.rumble);

	//always cancel auto init on reset, even if we don't reset the sticks
	controls.autoInit = 0;
	setAutoInitSetting(controls.autoInit);

	for(int extra=0; extra<EXTRAS_SIZE; extra++){
		ExtrasSlot slot = (ExtrasSlot)extra;
		for(int offset=0; offset<4; offset++){
			setExtrasSettingInt(slot, offset, 0);
		}
	}

	if(reset == HARD || reset == FACTORY){
		float notchAngles[_noOfNotches];
		for(int i = 0; i < _noOfNotches; i++){
			notchAngles[i] = _notchAngleDefaults[i];
		}
		setNotchAnglesSetting(notchAngles, ASTICK);
		setNotchAnglesSetting(notchAngles, CSTICK);

		setPointsSetting(_defaultCalPointsX, ASTICK, XAXIS);
		setPointsSetting(_defaultCalPointsY, ASTICK, YAXIS);
#ifdef ARDUINO
		Serial.println("A calibration points stored in EEPROM");
#endif //ARDUINO

		float cleanedPointsX[_noOfNotches+1];
		float cleanedPointsY[_noOfNotches+1];
		float notchPointsX[_noOfNotches+1];
		float notchPointsY[_noOfNotches+1];
		NotchStatus notchStatus[_noOfNotches];

		cleanCalPoints(_defaultCalPointsX, _defaultCalPointsY, notchAngles, cleanedPointsX, cleanedPointsY, notchPointsX, notchPointsY, notchStatus);
#ifdef ARDUINO
		Serial.println("A calibration points cleaned");
#endif //ARDUINO
		linearizeCal(cleanedPointsX, cleanedPointsY, cleanedPointsX, cleanedPointsY, aStickParams);
#ifdef ARDUINO
		Serial.println("A stick linearized");
#endif //ARDUINO
		notchCalibrate(cleanedPointsX, cleanedPointsY, notchPointsX, notchPointsY, _noOfNotches, aStickParams);

		setPointsSetting(_defaultCalPointsX, CSTICK, XAXIS);
		setPointsSetting(_defaultCalPointsY, CSTICK, YAXIS);
#ifdef ARDUINO
		Serial.println("C calibration points stored in EEPROM");
#endif //ARDUINO

		cleanCalPoints(_defaultCalPointsX, _defaultCalPointsY, notchAngles, cleanedPointsX, cleanedPointsY, notchPointsX, notchPointsY, notchStatus);
#ifdef ARDUINO
		Serial.println("C calibration points cleaned");
#endif //ARDUINO
		linearizeCal(cleanedPointsX, cleanedPointsY, cleanedPointsX, cleanedPointsY, cStickParams);
#ifdef ARDUINO
		Serial.println("C stick linearized");
#endif //ARDUINO
		notchCalibrate(cleanedPointsX, cleanedPointsY, notchPointsX, notchPointsY, _noOfNotches, cStickParams);
	}
#ifdef BATCHSETTINGS
	commitSettings(noLock);
#endif //BATCHSETTINGS
}

void copyButtons(const Buttons &src, Buttons &dest) {
	dest.A = src.A;
	dest.B = src.B;
	dest.X = src.X;
	dest.Y = src.Y;
	dest.S = src.S;
	dest.L = src.L;
	dest.R = src.R;
	dest.Z = src.Z;
	dest.Dr = src.Dr;
	dest.Du = src.Du;
	dest.Dl = src.Dl;
	dest.Dd = src.Dd;
	dest.La = src.La;
	dest.Ra = src.Ra;
}

void processButtons(Pins &pin, Buttons &btn, Buttons &hardware, ControlConfig &controls, FilterGains &gains, FilterGains &normGains, int &currentCalStep, bool &running, float tempCalPointsX[], float tempCalPointsY[], WhichStick &whichStick, NotchStatus notchStatus[], float notchAngles[], float measuredNotchAngles[], StickParams &aStickParams, StickParams &cStickParams){
	//Gather the button data from the hardware
	readButtons(pin, hardware);

	//Copy hardware buttons into a temp
	Buttons tempBtn;
	copyButtons(hardware, tempBtn);

	//Swap buttons here for jump remapping
	applyJump(controls, hardware, tempBtn);

	//read the L and R sliders here instead of readSticks so we don't get race conditions for mode 6

	//set up lockout for mode 5; it's not permissible to have analog trigger
	// inputs available while mode 5 is active
	//when a trigger is in lockout due to the other being mode 5,
	// modes 1, 3, and 4 will have no output on that trigger to warn the user.
	//(the above modes are 1-indexed, user-facing values)
	const bool lockoutL = controls.rConfig == 4;
	const bool lockoutR = controls.lConfig == 4;

	//We multiply the analog trigger reads by this to shut them off if the trigger is mapped to jump
	const int shutoffLa = (controls.jumpConfig == SWAP_XL || controls.jumpConfig == SWAP_YL) ? 0 : 1;
	const int shutoffRa = (controls.jumpConfig == SWAP_XR || controls.jumpConfig == SWAP_YR) ? 0 : 1;

	//These are used for mode 7, but they're calculated out here so we can scale the deadzone too.
	float triggerScaleL = (0.0112 * controls.lTriggerOffset) + 0.4494;
	float triggerScaleR = (0.0112 * controls.rTriggerOffset) + 0.4494;

	switch(controls.lConfig) {
		case 0: //Default Trigger state
			if(lockoutL){
				tempBtn.L  = (uint8_t) 0;
				tempBtn.La = (uint8_t) 0;
			} else {
				tempBtn.La = (uint8_t) readLa(pin, controls.lTrigInitial, 1) * shutoffLa;
			}
			break;
		case 1: //Digital Only Trigger state
			tempBtn.La = (uint8_t) 0;
			break;
		case 2: //Analog Only Trigger state
			if(lockoutL){
				tempBtn.L  = (uint8_t) 0;
				tempBtn.La = (uint8_t) 0;
			} else {
				tempBtn.L  = (uint8_t) 0;
				tempBtn.La = (uint8_t) readLa(pin, controls.lTrigInitial, 1) * shutoffLa;
			}
			break;
		case 3: //Trigger Plug Emulation state
			if(lockoutL){
				tempBtn.L  = (uint8_t) 0;
				tempBtn.La = (uint8_t) 0;
			} else {
				tempBtn.La = (uint8_t) readLa(pin, controls.lTrigInitial, 1) * shutoffLa;
				if (tempBtn.La > ((uint8_t) controls.lTriggerOffset)) {
					tempBtn.La = (uint8_t) controls.lTriggerOffset;
				}
			}
			break;
		case 4: //Digital => Analog Value state
			if(tempBtn.L) {
				tempBtn.La = (uint8_t) min(controls.lTriggerOffset, 255);
			} else {
				tempBtn.La = (uint8_t) 0;
			}
			tempBtn.L = (uint8_t) 0;
			break;
		case 5: //Digital => Analog Value + Digital state
			if(tempBtn.L) {
				tempBtn.La = (uint8_t) min(controls.lTriggerOffset, 255);
			} else {
				tempBtn.La = (uint8_t) 0;
			}
			break;
		case 6: //Scales Analog Trigger Values
			if(lockoutL){
				tempBtn.L  = (uint8_t) 0;
				tempBtn.La = (uint8_t) 0;
			} else {
				tempBtn.La = (uint8_t) readLa(pin, controls.lTrigInitial, triggerScaleL) * shutoffLa;
			}
			break;
		default:
			if(lockoutL){
				tempBtn.L  = (uint8_t) 0;
				tempBtn.La = (uint8_t) 0;
			} else {
				tempBtn.La = (uint8_t) readLa(pin, controls.lTrigInitial, 1) * shutoffLa;
			}
	}

	switch(controls.rConfig) {
		case 0: //Default Trigger state
			if(lockoutR){
				tempBtn.R  = (uint8_t) 0;
				tempBtn.Ra = (uint8_t) 0;
			} else {
				tempBtn.Ra = (uint8_t) readRa(pin, controls.rTrigInitial, 1) * shutoffRa;
			}
			break;
		case 1: //Digital Only Trigger state
			tempBtn.Ra = (uint8_t) 0;
			break;
		case 2: //Analog Only Trigger state
			if(lockoutR){
				tempBtn.R  = (uint8_t) 0;
				tempBtn.Ra = (uint8_t) 0;
			} else {
				tempBtn.R  = (uint8_t) 0;
				tempBtn.Ra = (uint8_t) readRa(pin, controls.rTrigInitial, 1) * shutoffRa;
			}
			break;
		case 3: //Trigger Plug Emulation state
			if(lockoutR){
				tempBtn.R  = (uint8_t) 0;
				tempBtn.Ra = (uint8_t) 0;
			} else {
				tempBtn.Ra = (uint8_t) readRa(pin, controls.rTrigInitial, 1) * shutoffRa;
				if (tempBtn.Ra > ((uint8_t) controls.rTriggerOffset)) {
					tempBtn.Ra = (uint8_t) controls.rTriggerOffset;
				}
			}
			break;
		case 4: //Digital => Analog Value state
			if(tempBtn.R) {
				tempBtn.Ra = (uint8_t) min(controls.rTriggerOffset, 255);
			} else {
				tempBtn.Ra = (uint8_t) 0;
			}
			tempBtn.R = (uint8_t) 0;
			break;
		case 5: //Digital => Analog Value + Digital state
			if(tempBtn.R) {
				tempBtn.Ra = (uint8_t) min(controls.rTriggerOffset, 255);
			} else {
				tempBtn.Ra = (uint8_t) 0;
			}
			break;
		case 6: //Scales Analog Trigger Values
			if(lockoutR){
				tempBtn.R  = (uint8_t) 0;
				tempBtn.Ra = (uint8_t) 0;
			} else {
				tempBtn.Ra = (uint8_t) readRa(pin, controls.rTrigInitial, triggerScaleR) * shutoffRa;
			}
			break;
		default:
			if(lockoutR){
				tempBtn.R  = (uint8_t) 0;
				tempBtn.Ra = (uint8_t) 0;
			} else {
				tempBtn.Ra = (uint8_t) readRa(pin, controls.rTrigInitial, 1) * shutoffRa;
			}
	}
	//Implement a small trigger deadzone of 3 units so that Dolphin initializes properly.
	//If we don't, then we can't trust that the waveshaping values display accurately
	// or that Mode 4 will stop at the right value on Dolphin.
	//Or on Smash Ult, maybe? No clue.
	if(tempBtn.La <= 3 * triggerScaleL){
		tempBtn.La = (uint8_t) 0;
	}
	if(tempBtn.Ra <= 3 * triggerScaleR){
		tempBtn.Ra = (uint8_t) 0;
	}

	//Apply any further button remapping to tempBtn here

	//Here we make sure LRAS actually operate.
	if(hardware.L && hardware.R && hardware.A && hardware.S) {
		tempBtn.L = (uint8_t) (1);
		tempBtn.R = (uint8_t) (1);
		tempBtn.A = (uint8_t) (1);
		tempBtn.S = (uint8_t) (1);
		tempBtn.La = (uint8_t) (255);
		tempBtn.Ra = (uint8_t) (255);
	}
	//Copy temp buttons (including analog triggers) back to btn
	copyButtons(tempBtn, btn);

	/* Current Commands List
	* Safe Mode:  AXY+Start
	* Display Version: AZ+Du
	*
	* Soft Reset:  ABZ+Start
	* Hard Reset:  ABZ+Dd
	* Auto-Initialize: AXY+Z
	*
	* Increase/Decrease Rumble: AB+Du/Dd
	* Show Current Rumble Setting: AB+Start
	*
	* Calibration
	* Analog Stick Calibration:  AXY+L
	* C-Stick Calibration:  AXY+R
	* Advance Calibration:  A or L or R
	* Undo Calibration:  Z
	* Skip to Notch Adjustment:  Start
	* Notch Adjustment CW/CCW:  X/Y
	* Notch Adjustment Reset:  B
	*
	* Analog Stick Configuration:
	* Increase/Decrease X-Axis Snapback Filtering:  AX+Du/Dd
	* Increase/Decrease Y-Axis Snapback Filtering:  AY+Du/Dd
	* Increase/Decrease X-Axis Waveshaping:  LX+Du/Dd
	* Increase/Decrease Y-Axis Waveshaping:  LY+Du/Dd
	* Increase/Decrease X-Axis Smoothing:  RX+Du/Dd
	* Increase/Decrease Y-Axis Smoothing:  RY+Du/Dd
	* Show Analog Filtering Settings: L+Start
	*
	* C-Stick Configuration
	* Increase/Decrease X-Axis Snapback Filtering:  AXZ+Du/Dd
	* Increase/Decrease Y-Axis Snapback Filtering:  AYZ+Du/Dd
	* Increase/Decrease X-Axis Waveshaping:  LXZ+Du/Dd
	* Increase/Decrease X-Axis Waveshaping:  LXZ+Du/Dd
	* Increase/Decrease X-Axis Offset:  RXZ+Du/Dd
	* Increase/Decrease Y-Axis Offset:  RYZ+Du/Dd
	* Show C-Stick Settings:  R+Start
	*
	* Swap X with Z:  XZ+Start
	* Swap Y with Z:  YZ+Start
	* Swap X with L:  LX+Start
	* Swap Y with L:  LY+Start
	* Swap X with R:  RX+Start
	* Swap Y with R:  Ry+Start
	*
	* Toggle L Trigger Mode:  AB+L
	* Toggle R Trigger Mode:  AB+R
	* Increase/Decrease L-trigger Offset:  LB+Du/Dd
	* Increase/Decrease R-Trigger Offset:  RB+Du/Dd
	*
	* Extras:
	* Toggle by holding both sticks in the chosen direction, then pressing A+B
	* Adjust by holding both sticks in the chosen direction, then pressing A+Dpad directions
	*/

	static bool advanceCal = false;

	//check the hardware buttons to change the controller settings
	if(!controls.safeMode && (currentCalStep == -1)) {
		static float hardResetAccumulator = 0.0;
		if (hardware.A && hardware.B && hardware.Z && hardware.Dd) { //Hard Reset pressed
			hardResetAccumulator = 0.99*hardResetAccumulator + 0.01;
		} else {
			hardResetAccumulator = 0.99*hardResetAccumulator;
		}
		if(hardResetAccumulator > 0.99) {
			hardResetAccumulator = 0;
			resetDefaults(HARD, controls, gains, normGains, _aStickParams, _cStickParams);//do reset sticks
			freezeSticks(2000, btn, hardware);
		}

		if(hardware.A && hardware.X && hardware.Y && hardware.S && !hardware.L && !hardware.R) { //Safe Mode Toggle
			controls.safeMode = true;
			freezeSticks(4000, btn, hardware);
		} else if (hardware.A && hardware.Z && hardware.Du && !hardware.X && !hardware.Y) { //display version number (ignore commands for c stick snapback)
			const int versionHundreds = floor(SW_VERSION/100.0);
			const int versionOnes     = SW_VERSION-versionHundreds;
			btn.Ax = (uint8_t) _floatOrigin;
			btn.Ay = (uint8_t) _floatOrigin;
			btn.Cx = (uint8_t) _floatOrigin + versionHundreds;
			btn.Cy = (uint8_t) _floatOrigin + versionOnes;
			clearButtons(2000, btn, hardware);
		} else if (hardware.A && hardware.B && hardware.Z && hardware.S) { //Soft Reset
			resetDefaults(SOFT, controls, gains, normGains, _aStickParams, _cStickParams);//don't reset sticks
			freezeSticks(2000, btn, hardware);
		} else if (hardware.A && hardware.B && hardware.Z && hardware.Dd) { //Hard Reset
			//actually do nothing, this is just to prevent othing things from happening
		} else if (hardware.A && hardware.X && hardware.Y && hardware.Z) { //Toggle Auto-Initialize
			changeAutoInit(btn, hardware, controls);
		} else if (hardware.A && hardware.B && hardware.Du) { //Increase Rumble
#ifdef RUMBLE
			changeRumble(INCREASE, btn, hardware, controls);
#else // RUMBLE
			//nothing
			freezeSticks(2000, btn, hardware);
#endif // RUMBLE
		} else if (hardware.A && hardware.B && hardware.Dd) { //Decrease Rumble
#ifdef RUMBLE
			changeRumble(DECREASE, btn, hardware, controls);
#else // RUMBLE
			//nothing
			freezeSticks(2000, btn, hardware);
#endif // RUMBLE
		} else if (hardware.A && hardware.B && hardware.S) { //Show current rumble setting
#ifdef RUMBLE
			showRumble(2000, btn, hardware, controls);
#else // RUMBLE
			freezeSticks(2000, btn, hardware);
#endif // RUMBLE
		} else if (hardware.A && hardware.X && hardware.Y && hardware.L) { //Analog Calibration
#ifdef ARDUINO
			Serial.println("Calibrating the A stick");
#endif //ARDUINO
			whichStick = ASTICK;
			currentCalStep ++;
			advanceCal = true;
			freezeSticks(2000, btn, hardware);
		} else if (hardware.A && hardware.X && hardware.Y && hardware.R) { //C-stick Calibration
#ifdef ARDUINO
			Serial.println("Calibrating the C stick");
#endif //ARDUINO
			whichStick = CSTICK;
			currentCalStep ++;
			advanceCal = true;
			freezeSticks(2000, btn, hardware);
		} else if(hardware.A && hardware.X && !hardware.Z && hardware.Du) { //Increase Analog X-Axis Snapback Filtering
			adjustSnapback(XAXIS, INCREASE, btn, hardware, controls, gains, normGains);
		} else if(hardware.A && hardware.X && !hardware.Z && hardware.Dd) { //Decrease Analog X-Axis Snapback Filtering
			adjustSnapback(XAXIS, DECREASE, btn, hardware, controls, gains, normGains);
		} else if(hardware.A && hardware.Y && !hardware.Z && hardware.Du) { //Increase Analog Y-Axis Snapback Filtering
			adjustSnapback(YAXIS, INCREASE, btn, hardware, controls, gains, normGains);
		} else if(hardware.A && hardware.Y && !hardware.Z && hardware.Dd) { //Decrease Analog Y-Axis Snapback Filtering
			adjustSnapback(YAXIS, DECREASE, btn, hardware, controls, gains, normGains);
		} else if(hardware.L && hardware.X && !hardware.Z && hardware.Du) { //Increase Analog X-Axis Waveshaping
			adjustWaveshaping(ASTICK, XAXIS, INCREASE, btn, hardware, controls);
		} else if(hardware.L && hardware.X && !hardware.Z && hardware.Dd) { //Decrease Analog X-Axis Waveshaping
			adjustWaveshaping(ASTICK, XAXIS, DECREASE, btn, hardware, controls);
		} else if(hardware.L && hardware.Y && !hardware.Z && hardware.Du) { //Increase Analog Y-Axis Waveshaping
			adjustWaveshaping(ASTICK, YAXIS, INCREASE, btn, hardware, controls);
		} else if(hardware.L && hardware.Y && !hardware.Z && hardware.Dd) { //Decrease Analog Y-Axis Waveshaping
			adjustWaveshaping(ASTICK, YAXIS, DECREASE, btn, hardware, controls);
		} else if(hardware.R && hardware.X && !hardware.Z && hardware.Du) { //Increase X-axis Delay
			adjustSmoothing(XAXIS, INCREASE, btn, hardware, controls, gains, normGains);
		} else if(hardware.R && hardware.X && !hardware.Z && hardware.Dd) { //Decrease X-axis Delay
			adjustSmoothing(XAXIS, DECREASE, btn, hardware, controls, gains, normGains);
		} else if(hardware.R && hardware.Y && !hardware.Z && hardware.Du) { //Increase Y-axis Delay
			adjustSmoothing(YAXIS, INCREASE, btn, hardware, controls, gains, normGains);
		} else if(hardware.R && hardware.Y && !hardware.Z && hardware.Dd) { //Decrease Y-axis Delay
			adjustSmoothing(YAXIS, DECREASE, btn, hardware, controls, gains, normGains);
		} else if(hardware.L && hardware.S && !hardware.A && !hardware.R && !hardware.X && !hardware.Y) { //Show Current Analog Settings (ignore L jump and L trigger toggle and LRAS)
			showAstickSettings(btn, hardware, controls, gains);
		} else if(hardware.A && hardware.X && hardware.Z && hardware.Du) { //Increase C-stick X-Axis Snapback Filtering
			adjustCstickSmoothing(XAXIS, INCREASE, btn, hardware, controls, gains, normGains);
		} else if(hardware.A && hardware.X && hardware.Z && hardware.Dd) { //Decrease C-stick X-Axis Snapback Filtering
			adjustCstickSmoothing(XAXIS, DECREASE, btn, hardware, controls, gains, normGains);
		} else if(hardware.A && hardware.Y && hardware.Z && hardware.Du) { //Increase C-stick Y-Axis Snapback Filtering
			adjustCstickSmoothing(YAXIS, INCREASE, btn, hardware, controls, gains, normGains);
		} else if(hardware.A && hardware.Y && hardware.Z && hardware.Dd) { //Decrease C-stick Y-Axis Snapback Filtering
			adjustCstickSmoothing(YAXIS, DECREASE, btn, hardware, controls, gains, normGains);
		} else if(hardware.L && hardware.X && hardware.Z && hardware.Du) { //Increase C-stick X-Axis Waveshaping
			adjustWaveshaping(CSTICK, XAXIS, INCREASE, btn, hardware, controls);
		} else if(hardware.L && hardware.X && hardware.Z && hardware.Dd) { //Decrease C-stick X-Axis Waveshaping
			adjustWaveshaping(CSTICK, XAXIS, DECREASE, btn, hardware, controls);
		} else if(hardware.L && hardware.Y && hardware.Z && hardware.Du) { //Increase C-stick Y-Axis Waveshaping
			adjustWaveshaping(CSTICK, YAXIS, INCREASE, btn, hardware, controls);
		} else if(hardware.L && hardware.Y && hardware.Z && hardware.Dd) { //Decrease C-stick Y-Axis Waveshaping
			adjustWaveshaping(CSTICK, YAXIS, DECREASE, btn, hardware, controls);
		} else if(hardware.R && hardware.X && hardware.Z && hardware.Du) { //Increase C-stick X Offset
			adjustCstickOffset(XAXIS, INCREASE, btn, hardware, controls);
		} else if(hardware.R && hardware.X && hardware.Z && hardware.Dd) { //Decrease C-stick X Offset
			adjustCstickOffset(XAXIS, DECREASE, btn, hardware, controls);
		} else if(hardware.R && hardware.Y && hardware.Z && hardware.Du) { //Increase C-stick Y Offset
			adjustCstickOffset(YAXIS, INCREASE, btn, hardware, controls);
		} else if(hardware.R && hardware.Y && hardware.Z && hardware.Dd) { //Decrease C-stick Y Offset
			adjustCstickOffset(YAXIS, DECREASE, btn, hardware, controls);
		} else if(hardware.R && hardware.S && !hardware.A && !hardware.L && !hardware.X && !hardware.Y) { //Show Current C-stick Settings (ignore R jump and R trigger toggle and LRAS)
			showCstickSettings(btn, hardware, controls, gains);
		} else if(hardware.A && hardware.B && hardware.L) { //Toggle Analog L
			nextTriggerState(LTRIGGER, btn, hardware, controls);
		} else if(hardware.A && hardware.B && hardware.R) { //Toggle Analog R
			nextTriggerState(RTRIGGER, btn, hardware, controls);
		} else if(hardware.L && hardware.B && hardware.Du) { //Increase L-Trigger Offset
			adjustTriggerOffset(LTRIGGER, INCREASE, btn, hardware, controls);
		} else if(hardware.L && hardware.B && hardware.Dd) { //Decrease L-trigger Offset
			adjustTriggerOffset(LTRIGGER, DECREASE, btn, hardware, controls);
		} else if(hardware.R && hardware.B && hardware.Du) { //Increase R-trigger Offset
			adjustTriggerOffset(RTRIGGER, INCREASE, btn, hardware, controls);
		} else if(hardware.R && hardware.B && hardware.Dd) { //Decrease R-trigger Offset
			adjustTriggerOffset(RTRIGGER, DECREASE, btn, hardware, controls);
		} else if(hardware.X && hardware.Z && hardware.S) { //Swap X and Z
			setJumpConfig(SWAP_XZ, controls);
			freezeSticks(2000, btn, hardware);
		} else if(hardware.Y && hardware.Z && hardware.S) { //Swap Y and Z
			setJumpConfig(SWAP_YZ, controls);
			freezeSticks(2000, btn, hardware);
		} else if(hardware.X && hardware.L && hardware.S) { //Swap X and L
			setJumpConfig(SWAP_XL, controls);
			freezeSticks(2000, btn, hardware);
		} else if(hardware.Y && hardware.L && hardware.S) { //Swap Y and L
			setJumpConfig(SWAP_YL, controls);
			freezeSticks(2000, btn, hardware);
		} else if(hardware.X && hardware.R && hardware.S) { //Swap X and R
			setJumpConfig(SWAP_XR, controls);
			freezeSticks(2000, btn, hardware);
		} else if(hardware.Y && hardware.R && hardware.S) { //Swap Y and R
			setJumpConfig(SWAP_YR, controls);
			freezeSticks(2000, btn, hardware);
		} else if(checkAdjustExtra(EXTRAS_UP, btn, false)) { // Toggle Extras
			toggleExtra(EXTRAS_UP, btn, hardware, controls);
		} else if(checkAdjustExtra(EXTRAS_DOWN, btn, false)) {
			toggleExtra(EXTRAS_DOWN, btn, hardware, controls);
		} else if(checkAdjustExtra(EXTRAS_LEFT, btn, false)) {
			toggleExtra(EXTRAS_LEFT, btn, hardware, controls);
		} else if(checkAdjustExtra(EXTRAS_RIGHT, btn, false)) {
			toggleExtra(EXTRAS_RIGHT, btn, hardware, controls);
		} else if(checkAdjustExtra(EXTRAS_UP, btn, true)) { // Configure Extras
			configExtra(EXTRAS_UP, btn, hardware, controls);
		} else if(checkAdjustExtra(EXTRAS_DOWN, btn, true)) {
			configExtra(EXTRAS_DOWN, btn, hardware, controls);
		} else if(checkAdjustExtra(EXTRAS_LEFT, btn, true)) {
			configExtra(EXTRAS_LEFT, btn, hardware, controls);
		} else if(checkAdjustExtra(EXTRAS_RIGHT, btn, true)) {
			configExtra(EXTRAS_RIGHT, btn, hardware, controls);
		}
	} else if (currentCalStep == -1) { //Safe Mode Enabled, Lock Settings, wait for safe mode command
		static float safeModeAccumulator = 0.0;
		if(hardware.A && hardware.X && hardware.Y && hardware.S && !hardware.L && !hardware.R) { //Safe Mode Toggle
			safeModeAccumulator = 0.99*safeModeAccumulator + 0.01;
		} else {
			safeModeAccumulator = 0.99*safeModeAccumulator;
		}
		if(safeModeAccumulator > 0.99){
			safeModeAccumulator = 0;
			if (!running) {//wake it up if not already running
				running = true;
			}
			controls.safeMode = false;
			freezeSticks(2000, btn, hardware);
		}
	}


	//Skip stick measurement and go to notch adjust using the start button while calibrating
	if(hardware.S && (currentCalStep >= 0 && currentCalStep < 32)){
		currentCalStep = _noOfCalibrationPoints;
		//Do the same thing we would have done at step 32 had we actually collected the points, but with stored tempCalPoints
		if(whichStick == CSTICK){
			//get the calibration points collected during the last stick calibration
			getPointsSetting(tempCalPointsX, whichStick, XAXIS);
			getPointsSetting(tempCalPointsY, whichStick, YAXIS);
			applyCalFromPoints(whichStick, notchAngles, tempCalPointsX, tempCalPointsY, notchStatus, measuredNotchAngles, cStickParams);
		} else if(whichStick == ASTICK){
			//get the calibration points collected during the last stick calibration
			getPointsSetting(tempCalPointsX, whichStick, XAXIS);
			getPointsSetting(tempCalPointsY, whichStick, YAXIS);
			applyCalFromPoints(whichStick, notchAngles, tempCalPointsX, tempCalPointsY, notchStatus, measuredNotchAngles, aStickParams);
		}
	}
	//Undo Calibration using Z-button
	static bool undoCal = false;
	static bool undoCalPressed = false;
	if(hardware.Z && undoCal && !undoCalPressed) {
		undoCalPressed = true;
		if(currentCalStep % 2 == 0 && currentCalStep < 32 && currentCalStep != 0 ) {
			//If it's measuring zero, go back to the previous zero
			currentCalStep --;
			currentCalStep --;
		} else if(currentCalStep % 2 == 1 && currentCalStep < 32 && currentCalStep != 0 ) {
			//If it's measuring a notch, go back to the zero before the previous notch
			currentCalStep -= 3;
			currentCalStep = max(currentCalStep, 0);
		} else if(currentCalStep > 32) {
			//We can go directly between notches when adjusting notches
			currentCalStep --;
		}
		if(whichStick == CSTICK){
			int notchIndex = _notchAdjOrder[min(currentCalStep-_noOfCalibrationPoints, _noOfAdjNotches-1)];//limit this so it doesn't access outside the array bounds
			while((currentCalStep >= _noOfCalibrationPoints) && (notchStatus[notchIndex] == TERT_INACTIVE) && (currentCalStep < _noOfCalibrationPoints + _noOfAdjNotches)){//this non-diagonal notch was not calibrated
				//skip to the next valid notch
				currentCalStep--;
				notchIndex = _notchAdjOrder[min(currentCalStep-_noOfCalibrationPoints, _noOfAdjNotches-1)];//limit this so it doesn't access outside the array bounds
			}
		} else if(whichStick == ASTICK){
			int notchIndex = _notchAdjOrder[min(currentCalStep-_noOfCalibrationPoints, _noOfAdjNotches-1)];//limit this so it doesn't access outside the array bounds
			while((currentCalStep >= _noOfCalibrationPoints) && (notchStatus[notchIndex] == TERT_INACTIVE) && (currentCalStep < _noOfCalibrationPoints + _noOfAdjNotches)){//this non-diagonal notch was not calibrated
				//skip to the next valid notch
				currentCalStep--;
				notchIndex = _notchAdjOrder[min(currentCalStep-_noOfCalibrationPoints, _noOfAdjNotches-1)];//limit this so it doesn't access outside the array bounds
			}
		}
	} else if(!hardware.Z) {
		undoCalPressed = false;
	}

	//Advance Calibration Using L or R triggers
	static float advanceCalAccumulator = 0.0;
	if((hardware.A || hardware.L || hardware.R) && advanceCal){
		advanceCalAccumulator = 0.96*advanceCalAccumulator + 0.04;
	} else {
		advanceCalAccumulator = 0.96*advanceCalAccumulator;
	}

	static bool advanceCalPressed = false;
	if(advanceCalAccumulator > 0.75 && !advanceCalPressed){
		advanceCalPressed = true;
		if (whichStick == CSTICK){
			if(currentCalStep < _noOfCalibrationPoints){//still collecting points
				readADCScale(_ADCScale, _ADCScaleFactor);
				float X = 0;
				float Y = 0;
				for(int i=0; i<128; i++) {
					X += readCx(_pinList)/4096.0*_ADCScale;
					Y += readCy(_pinList)/4096.0*_ADCScale;
				}
				X /= 128.0;
				Y /= 128.0;
				insertCalPoints(whichStick, currentCalStep, tempCalPointsX, tempCalPointsY, _pinList, X, Y);
			}
			currentCalStep ++;
			if(currentCalStep >= 2 && currentCalStep != _noOfNotches*2) {//don't undo at the beginning of collection or notch adjust
				undoCal = true;
			} else {
				undoCal = false;
			}
			if(currentCalStep == _noOfCalibrationPoints){//done collecting points
				applyCalFromPoints(whichStick, notchAngles, tempCalPointsX, tempCalPointsY, notchStatus, measuredNotchAngles, cStickParams);
			}
			int notchIndex = _notchAdjOrder[min(currentCalStep-_noOfCalibrationPoints, _noOfAdjNotches-1)];//limit this so it doesn't access outside the array bounds
			while((currentCalStep >= _noOfCalibrationPoints) && (notchStatus[notchIndex] == TERT_INACTIVE) && (currentCalStep < _noOfCalibrationPoints + _noOfAdjNotches)){//this non-diagonal notch was not calibrated
				//skip to the next valid notch
				currentCalStep++;
				notchIndex = _notchAdjOrder[min(currentCalStep-_noOfCalibrationPoints, _noOfAdjNotches-1)];//limit this so it doesn't access outside the array bounds
			}
			if(currentCalStep >= _noOfCalibrationPoints + _noOfAdjNotches){//done adjusting notches
#ifdef ARDUINO
				Serial.println("finished adjusting notches for the C stick");
#endif //ARDUINO
				setPointsSetting(tempCalPointsX, whichStick, XAXIS);
				setPointsSetting(tempCalPointsY, whichStick, YAXIS);
				setNotchAnglesSetting(notchAngles, whichStick);
				controls.autoInit = 0;
				setAutoInitSetting(controls.autoInit);
#ifdef BATCHSETTINGS
				commitSettings();
#endif //BATCHSETTINGS
#ifdef ARDUINO
				Serial.println("calibration points stored in EEPROM");
#endif //ARDUINO
				float cleanedPointsX[_noOfNotches+1];
				float cleanedPointsY[_noOfNotches+1];
				float notchPointsX[_noOfNotches+1];
				float notchPointsY[_noOfNotches+1];
				cleanCalPoints(tempCalPointsX, tempCalPointsY, notchAngles, cleanedPointsX, cleanedPointsY, notchPointsX, notchPointsY, notchStatus);
#ifdef ARDUINO
				Serial.println("calibration points cleaned");
#endif //ARDUINO
				linearizeCal(cleanedPointsX, cleanedPointsY, cleanedPointsX, cleanedPointsY, cStickParams);
#ifdef ARDUINO
				Serial.println("C stick linearized");
#endif //ARDUINO
				notchCalibrate(cleanedPointsX, cleanedPointsY, notchPointsX, notchPointsY, _noOfNotches, cStickParams);
				currentCalStep = -1;
				advanceCal = false;
			}
		}
		else if (whichStick == ASTICK){
#ifdef ARDUINO
			Serial.println("Current step:");
			Serial.println(currentCalStep);
#endif //ARDUINO
			if(currentCalStep < _noOfCalibrationPoints){//still collecting points
				readADCScale(_ADCScale, _ADCScaleFactor);
				float X = 0;
				float Y = 0;
				for(int i=0; i<128; i++) {
					X += readAx(_pinList)/4096.0*_ADCScale;
					Y += readAy(_pinList)/4096.0*_ADCScale;
				}
				X /= 128.0;
				Y /= 128.0;
				insertCalPoints(whichStick, currentCalStep, tempCalPointsX, tempCalPointsY, _pinList, X, Y);
			}
			currentCalStep ++;
			if(currentCalStep >= 2 && currentCalStep != _noOfCalibrationPoints) {//don't undo at the beginning of collection or notch adjust
				undoCal = true;
			} else {
				undoCal = false;
			}
			if(currentCalStep == _noOfCalibrationPoints){//done collecting points
				applyCalFromPoints(whichStick, notchAngles, tempCalPointsX, tempCalPointsY, notchStatus, measuredNotchAngles, aStickParams);
			}
			int notchIndex = _notchAdjOrder[min(currentCalStep-_noOfCalibrationPoints, _noOfAdjNotches-1)];//limit this so it doesn't access outside the array bounds
			while((currentCalStep >= _noOfCalibrationPoints) && (notchStatus[notchIndex] == TERT_INACTIVE) && (currentCalStep < _noOfCalibrationPoints + _noOfAdjNotches)){//this non-diagonal notch was not calibrated
				//skip to the next valid notch
				currentCalStep++;
				notchIndex = _notchAdjOrder[min(currentCalStep-_noOfCalibrationPoints, _noOfAdjNotches-1)];//limit this so it doesn't access outside the array bounds
			}
			if(currentCalStep >= _noOfCalibrationPoints + _noOfAdjNotches){//done adjusting notches
#ifdef ARDUINO
				Serial.println("finished adjusting notches for the A stick");
#endif //ARDUINO
				setPointsSetting(tempCalPointsX, whichStick, XAXIS);
				setPointsSetting(tempCalPointsY, whichStick, YAXIS);
				setNotchAnglesSetting(notchAngles, whichStick);
				controls.autoInit = 0;
				setAutoInitSetting(controls.autoInit);
#ifdef BATCHSETTINGS
				commitSettings();
#endif //BATCHSETTINGS
#ifdef ARDUINO
				Serial.println("calibration points stored in EEPROM");
#endif //ARDUINO
				float cleanedPointsX[_noOfNotches+1];
				float cleanedPointsY[_noOfNotches+1];
				float notchPointsX[_noOfNotches+1];
				float notchPointsY[_noOfNotches+1];
				cleanCalPoints(tempCalPointsX, tempCalPointsY, notchAngles, cleanedPointsX, cleanedPointsY, notchPointsX, notchPointsY, notchStatus);
#ifdef ARDUINO
				Serial.println("calibration points cleaned");
#endif //ARDUINO
				linearizeCal(cleanedPointsX, cleanedPointsY, cleanedPointsX, cleanedPointsY, aStickParams);
#ifdef ARDUINO
				Serial.println("A stick linearized");
#endif //ARDUINO
				notchCalibrate(cleanedPointsX, cleanedPointsY, notchPointsX, notchPointsY, _noOfNotches, aStickParams);
				currentCalStep = -1;
				advanceCal = false;
			}
		}
	} else if(advanceCalAccumulator <= 0.25) {
		advanceCalPressed = false;
	}
}

void readSticks(int readA, int readC, Buttons &btn, Pins &pin, RawStick &raw, const Buttons &hardware, const ControlConfig &controls, const FilterGains &normGains, const StickParams &aStickParams, const StickParams &cStickParams, float &dT, int &currentCalStep, const bool runSynced){
	readADCScale(_ADCScale, _ADCScaleFactor);



	//on Arduino (and therefore Teensy), micros() overflows after about 71.58 minutes
	//This is 2^32 microseconds
	static uint32_t lastMicros = micros();
	static uint32_t adcDelta = 0;
	//We increment lastMicros by 1000 each timestep so that we can always try to catch up
	// to the right timestep instead of always being late
	//If the timestep goes past 2^32, then we subtract 2^32.
	//However, this may make it smaller than the most recently measured time.
	//So, we let the loop keep going if

#ifndef CLEANADC
	//Read the sticks repeatedly until it's been 1 millisecond since the last iteration
	//This is for denoising and making sure the loop runs at 1000 Hz
	//We want to stop the ADC reading early enough that we don't overrun 1000 microseconds
	uint32_t adcCount = 0;
	uint32_t aXSum = 0;
	uint32_t aYSum = 0;
	uint32_t cXSum = 0;
	uint32_t cYSum = 0;
	uint32_t beforeMicros = micros();
	uint32_t afterMicros;
	do{
		adcCount++;
		aXSum += readAx(pin);
		aYSum += readAy(pin);
		cXSum += readCx(pin);
		cYSum += readCy(pin);
		afterMicros = micros();
		adcDelta = afterMicros-beforeMicros;
		beforeMicros = afterMicros;
	}
	while((afterMicros-lastMicros < 1000 - adcDelta) || (afterMicros-lastMicros > 100000000));

	//Then we spinlock to get the 1 kHz more exactly.
	while(afterMicros-lastMicros < 1000) {
		afterMicros = micros();
	}

	//Serial.println(adcCount);
	float aStickX = aXSum/(float)adcCount/4096.0*_ADCScale;
	float aStickY = aYSum/(float)adcCount/4096.0*_ADCScale;
	float cStickX = cXSum/(float)adcCount/4096.0*_ADCScale;
	float cStickY = cYSum/(float)adcCount/4096.0*_ADCScale;
#else //CLEANADC: read only once
	float aStickX = readAx(pin)/4096.0;
	float aStickY = readAy(pin)/4096.0;
	float cStickX = readCx(pin)/4096.0;
	float cStickY = readCy(pin)/4096.0;
	//note: this actually results in about 0.5 ms delay for the analog sticks

	if(!runSynced) {
		uint32_t thisMicros = micros();
		while(thisMicros-lastMicros < 1000) {
			thisMicros = micros();
		}
	}
#endif //CLEANADC
	if(!runSynced) {
		dT = 1;
		lastMicros += 1000;
	} else {
		//we the dT variable so that we know how long it actually took to cycle
		dT = (micros()-lastMicros)/1000;
		lastMicros = micros();
	}
	if(micros() - lastMicros > 1500) { //handle the case that it was synced and now isn't
		lastMicros = micros();
	}
	_raw.axRaw = aStickX;
	_raw.ayRaw = aStickY;
	_raw.cxRaw = cStickX;
	_raw.cyRaw = cStickY;

	//create the measurement value to be used in the kalman filter
	float xZ;
	float yZ;

	//linearize the analog stick inputs by multiplying by the coefficients found during calibration (3rd order fit)
	xZ = linearize(aStickX, aStickParams.fitCoeffsX);
	yZ = linearize(aStickY, aStickParams.fitCoeffsY);

	float posCx = linearize(cStickX, cStickParams.fitCoeffsX);
	float posCy = linearize(cStickY, cStickParams.fitCoeffsY);

	float posAx = xZ;
	float posAy = yZ;

	_raw.axLinearized = posAx;
	_raw.ayLinearized = posAy;
	_raw.cxLinearized = posCx;
	_raw.cyLinearized = posCy;

	//Run the kalman filter to eliminate snapback
	static float xPosFilt = 0;//output of kalman filter
	static float yPosFilt = 0;//output of kalman filter
	runKalman(xPosFilt, yPosFilt, xZ, yZ, controls, normGains);

	float shapedAx = xPosFilt;
	float shapedAy = yPosFilt;
	//Run waveshaping, a secondary filter to extend time at the rim
	aRunWaveShaping(shapedAx, shapedAy, shapedAx, shapedAy, controls, normGains);

	//Run a simple low-pass filter
	static float oldPosAx = 0;
	static float oldPosAy = 0;
	posAx = normGains.xSmoothing*shapedAx + (1-normGains.xSmoothing)*oldPosAx;
	posAy = normGains.ySmoothing*shapedAy + (1-normGains.ySmoothing)*oldPosAy;
	oldPosAx = posAx;
	oldPosAy = posAy;

	//Run waveshaping on the c-stick
	cRunWaveShaping(posCx, posCy, posCx, posCy, controls, normGains);

	//Run a simple low-pass filter on the C-stick
	static float cXPos = 0;
	static float cYPos = 0;
	float oldCX = cXPos;
	float oldCY = cYPos;
	cXPos = posCx;
	cYPos = posCy;
	float xWeight1 = normGains.cXSmoothing;
	float xWeight2 = 1-xWeight1;
	float yWeight1 = normGains.cYSmoothing;
	float yWeight2 = 1-yWeight1;

	cXPos = xWeight1*cXPos + xWeight2*oldCX;
	cYPos = yWeight1*cYPos + yWeight2*oldCY;

	posCx = cXPos;
	posCy = cYPos;

	//Run a median filter to reduce noise
#ifdef USEMEDIAN
	static float xPosList[MEDIANLEN] = MEDIANARRAY;//for median filtering;
	static float yPosList[MEDIANLEN] = MEDIANARRAY;//for median filtering
	static unsigned int xMedianIndex = 0;
	static unsigned int yMedianIndex = 0;
    runMedian(posAx, xPosList, xMedianIndex);
    runMedian(posAy, yPosList, yMedianIndex);
#endif

	float remappedAx;
	float remappedAy;
	float remappedCx;
	float remappedCy;
	float remappedAxUnfiltered;
	float remappedAyUnfiltered;
	float remappedCxUnfiltered;
	float remappedCyUnfiltered;
	notchRemap(posAx, posAy, &remappedAx, &remappedAy, _noOfNotches, aStickParams, currentCalStep);
	notchRemap(posCx, posCy, &remappedCx, &remappedCy, _noOfNotches, cStickParams, currentCalStep);
	notchRemap(_raw.axLinearized, _raw.ayLinearized, &remappedAxUnfiltered, &remappedAyUnfiltered, _noOfNotches, aStickParams, 1);//no snapping
	notchRemap(_raw.cxLinearized, _raw.cyLinearized, &remappedCxUnfiltered, &remappedCyUnfiltered, _noOfNotches, cStickParams, 1);//no snapping

	//Clamp values from -125 to +125
	remappedAx = fmin(125, fmax(-125, remappedAx));
	remappedAy = fmin(125, fmax(-125, remappedAy));
	remappedCx = fmin(125, fmax(-125, remappedCx+controls.cXOffset));
	remappedCy = fmin(125, fmax(-125, remappedCy+controls.cYOffset));
	_raw.axUnfiltered = fmin(125, fmax(-125, remappedAxUnfiltered));
	_raw.ayUnfiltered = fmin(125, fmax(-125, remappedAyUnfiltered));
	_raw.cxUnfiltered = fmin(125, fmax(-125, remappedCxUnfiltered+controls.cXOffset));
	_raw.cyUnfiltered = fmin(125, fmax(-125, remappedCyUnfiltered+controls.cYOffset));

	bool skipAHyst = false;
#ifdef EXTRAS_ESS
	//ESS adapter functionality for Ocarina of Time on WiiVC if enabled
	skipAHyst = ess::remap(&remappedAx, &remappedAy, controls.extras[ess::extrasEssConfigSlot].config);
#endif

	float hystVal = 0.3;
	//assign the remapped values to the button struct
	if(readA){
		if (!skipAHyst) {
			float diffAx = (remappedAx+_floatOrigin)-btn.Ax;
			if( (diffAx > (1.0 + hystVal)) || (diffAx < -hystVal) ){
				btn.Ax = (uint8_t) (remappedAx+_floatOrigin);
			}
			float diffAy = (remappedAy+_floatOrigin)-btn.Ay;
			if( (diffAy > (1.0 + hystVal)) || (diffAy < -hystVal) ){
				btn.Ay = (uint8_t) (remappedAy+_floatOrigin);
			}
		} else {
			btn.Ax = (uint8_t) (remappedAx+_floatOrigin);
			btn.Ay = (uint8_t) (remappedAy+_floatOrigin);
		}
	}
	if(readC){
		float diffCx = (remappedCx+_floatOrigin)-btn.Cx;
		if( (diffCx > (1.0 + hystVal)) || (diffCx < -hystVal) ){
			btn.Cx = (uint8_t) (remappedCx+_floatOrigin);
		}
		float diffCy = (remappedCy+_floatOrigin)-btn.Cy;
		if( (diffCy > (1.0 + hystVal)) || (diffCy < -hystVal) ){
			btn.Cy = (uint8_t) (remappedCy+_floatOrigin);
		}
	}
};

#endif //PHOBGCC_H
