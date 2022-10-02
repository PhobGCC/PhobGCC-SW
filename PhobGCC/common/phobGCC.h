#ifndef PHOBGCC_H
#define PHOBGCC_H

//Must be included at the end of the board header file, but before comms.h

#include "structsAndEnums.h"
#include "filter.h"
#include "stick.h"

//Uncomment the appropriate #include line for your hardware by deleting the two slashes at the beginning of the line.
//#include "../teensy/Phob1_0Teensy3_2.h"          // For PhobGCC board 1.0 with Teensy 3.2
//#include "../teensy/Phob1_0Teensy3_2DiodeShort.h"// For PhobGCC board 1.0 with Teensy 3.2 and the diode shorted
//#include "../teensy/Phob1_1Teensy3_2.h"          // For PhobGCC board 1.1 with Teensy 3.2
//#include "../teensy/Phob1_1Teensy3_2DiodeShort.h"// For PhobGCC board 1.1 with Teensy 3.2 and the diode shorted
//#include "../teensy/Phob1_1Teensy4_0.h"          // For PhobGCC board 1.1 with Teensy 4.0
//#include "../teensy/Phob1_1Teensy4_0DiodeShort.h"// For PhobGCC board 1.1 with Teensy 4.0 and the diode shorted
#include "../teensy/Phob1_2Teensy4_0.h"          // For PhobGCC board 1.2.x with Teensy 4.0

//#define BUILD_RELEASE
#define BUILD_DEV

//This is just an integer.
#define SW_VERSION 26

//#define ENABLE_LED

ControlConfig _controls{
	.pinXSwappable = _pinX,
	.pinYSwappable = _pinY,
	.pinZSwappable = _pinZ,
	.jumpConfig = DEFAULTJUMP,
	.jumpConfigMin = DEFAULTJUMP,
	.jumpConfigMax = SWAP_YZ,
	.lConfig = 0,
	.rConfig = 0,
	.triggerConfigMin = 0,
	.triggerConfigMax = 5,
	.triggerDefault = 0,
	.lTriggerOffset = 49,
	.rTriggerOffset = 49,
	.triggerMin = 49,
	.triggerMax = 227,
	.cXOffset = 0,
	.cYOffset = 0,
	.cMax = 127,
	.cMin = -127,
	.rumble = 5,
	.rumbleMin = 0,
	.rumbleMax = 7,
	.rumbleDefault = 5,
	.safeMode = true,
	.autoInit = false,
	.lTrigInitial = 0,
	.rTrigInitial = 0,
	.xSnapback = 4,
	.ySnapback = 4,
	.snapbackMin = 0,
	.snapbackMax = 10,
	.snapbackDefault = 4,
	.smoothingMin = 0,
	.smoothingMax = 0.9
};

FilterGains _gains {//these values are for 800 hz, recomputeGains converts them to what is needed for the actual frequency
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

Pins _pinList;


int calcRumblePower(const int rumble){
	if(rumble > 0) {
		return pow(2.0, 7+((rumble+1)/8.0)); //should be 256 when rumble is 7
	} else {
		return 0;
	}
}

float velDampFromSnapback(const int snapback){
	return 0.125 * pow(2, (snapback-4)/3.0);//4 should yield 0.125, 10 should yield 0.5, don't care about 0
}

void freezeSticks(const int time, Buttons &btn, HardwareButtons &hardware) {
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
void clearButtons(const int time, Buttons &btn, HardwareButtons &hardware) {
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

void showRumble(const int time, Buttons &btn, HardwareButtons &hardware, ControlConfig &controls) {
	btn.Cx = (uint8_t) _intOrigin;
	btn.Cy = (uint8_t) (controls.rumble + _floatOrigin);
	clearButtons(time, btn, hardware);

	setRumbleSetting(controls.rumble);
}

void changeRumble(const Increase increase, Buttons &btn, HardwareButtons &hardware, ControlConfig &controls) {
	Serial.println("changing rumble");
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
	showRumble(1000, btn, hardware, controls);
}

//Make it so you don't need to press B.
//This is only good if the sticks are calibrated, so
// the setting auto-resets whenever you hard reset or recalibrate.
void changeAutoInit(Buttons &btn, HardwareButtons &hardware, ControlConfig &controls) {
	if(controls.autoInit == 0) {
		controls.autoInit = 1;
	} else {
		controls.autoInit = 0;
	}

	//move sticks up-right for on, down-left for off
	btn.Ax = (uint8_t) (controls.autoInit*100 - 50 + _floatOrigin);
	btn.Ay = (uint8_t) (controls.autoInit*100 - 50 + _floatOrigin);
	btn.Cx = (uint8_t) (controls.autoInit*100 - 50 + _floatOrigin);
	btn.Cy = (uint8_t) (controls.autoInit*100 - 50 + _floatOrigin);

	clearButtons(2000, btn, hardware);

	setAutoInitSetting(controls.autoInit);
}

void adjustSnapback(const WhichAxis axis, const Increase increase, Buttons &btn, HardwareButtons &hardware, ControlConfig &controls, FilterGains &gains, FilterGains &normGains){
	Serial.println("adjusting snapback filtering");
	if(axis == XAXIS && increase == INCREASE){
		controls.xSnapback = min(controls.xSnapback+1, controls.snapbackMax);
		Serial.print("x snapback filtering increased to:");
		Serial.println(controls.xSnapback);
	}
	else if(axis == XAXIS && increase == DECREASE){
		controls.xSnapback = max(controls.xSnapback-1, controls.snapbackMin);
		Serial.print("x snapback filtering decreased to:");
		Serial.println(controls.xSnapback);
	}

	if(axis == YAXIS && increase == INCREASE){
		controls.ySnapback = min(controls.ySnapback+1, controls.snapbackMax);
		Serial.print("y snapback filtering increased to:");
		Serial.println(controls.ySnapback);
	}
	else if(axis == YAXIS && increase == DECREASE){
		controls.ySnapback = max(controls.ySnapback-1, controls.snapbackMin);
		Serial.print("y snapback filtering decreased to:");
		Serial.println(controls.ySnapback);
	}

	gains.xVelDamp = velDampFromSnapback(controls.xSnapback);
	gains.yVelDamp = velDampFromSnapback(controls.ySnapback);

    //recompute the intermediate gains used directly by the kalman filter
    recomputeGains(gains, normGains);

	btn.Cx = (uint8_t) (controls.xSnapback + _floatOrigin);
	btn.Cy = (uint8_t) (controls.ySnapback + _floatOrigin);

	clearButtons(2000, btn, hardware);

	setXSnapbackSetting(controls.xSnapback);
	setYSnapbackSetting(controls.ySnapback);
}

void adjustSmoothing(const WhichAxis axis, const Increase increase, Buttons &btn, HardwareButtons &hardware, ControlConfig &controls, FilterGains &gains, FilterGains &normGains) {
	Serial.println("Adjusting Smoothing");
	if (axis == XAXIS && increase == INCREASE) {
		gains.xSmoothing = gains.xSmoothing + 0.1;
		if(gains.xSmoothing > controls.smoothingMax) {
			gains.xSmoothing = controls.smoothingMax;
		}
		setXSmoothingSetting(gains.xSmoothing);
		Serial.print("X Smoothing increased to:");
		Serial.println(gains.xSmoothing);
	} else if(axis == XAXIS && increase == DECREASE) {
		gains.xSmoothing = gains.xSmoothing - 0.1;
		if(gains.xSmoothing < controls.smoothingMin) {
			gains.xSmoothing = controls.smoothingMin;
		}
		setXSmoothingSetting(gains.xSmoothing);
		Serial.print("X Smoothing decreased to:");
		Serial.println(gains.xSmoothing);
	} else if(axis == YAXIS && increase == INCREASE) {
		gains.ySmoothing = gains.ySmoothing + 0.1;
		if (gains.ySmoothing > controls.smoothingMax) {
			gains.ySmoothing = controls.smoothingMax;
		}
		setYSmoothingSetting(gains.ySmoothing);
		Serial.print("Y Smoothing increased to:");
		Serial.println(gains.ySmoothing);
	} else if(axis == YAXIS && increase == DECREASE) {
		gains.ySmoothing = gains.ySmoothing - 0.1;
		if (gains.ySmoothing < controls.smoothingMin) {
			gains.ySmoothing = controls.smoothingMin;
		}
		setYSmoothingSetting(gains.ySmoothing);
		Serial.print("Y Smoothing decreased to:");
		Serial.println(gains.ySmoothing);
	}

	//recompute the intermediate gains used directly by the kalman filter
	recomputeGains(gains, normGains);

	btn.Cx = (uint8_t) (_floatOrigin + (gains.xSmoothing * 10));
	btn.Cy = (uint8_t) (_floatOrigin + (gains.ySmoothing * 10));

	clearButtons(2000, btn, hardware);
}

void showAstickSettings(Buttons &btn, HardwareButtons &hardware, const ControlConfig &controls, FilterGains &gains) {
	//Snapback on A-stick
	btn.Ax = (uint8_t) (controls.xSnapback + _floatOrigin);
	btn.Ay = (uint8_t) (controls.ySnapback + _floatOrigin);

	//Smoothing on C-stick
	btn.Cx = (uint8_t) (_floatOrigin + (gains.xSmoothing * 10));
	btn.Cy = (uint8_t) (_floatOrigin + (gains.ySmoothing * 10));

	clearButtons(2000, btn, hardware);
}

void adjustCstickSmoothing(const WhichAxis axis, const Increase increase, Buttons &btn, HardwareButtons &hardware, ControlConfig &controls, FilterGains &gains, FilterGains &normGains) {
	Serial.println("Adjusting C-Stick Smoothing");
	if (axis == XAXIS && increase == INCREASE) {
		gains.cXSmoothing = gains.cXSmoothing + 0.1;
		if(gains.cXSmoothing > controls.smoothingMax) {
			gains.cXSmoothing = controls.smoothingMax;
		}
		setCxSmoothingSetting(gains.cXSmoothing);
		Serial.print("C-Stick X Smoothing increased to:");
		Serial.println(gains.cXSmoothing);
	} else if(axis == XAXIS && increase == DECREASE) {
		gains.cXSmoothing = gains.cXSmoothing - 0.1;
		if(gains.cXSmoothing < controls.smoothingMin) {
			gains.cXSmoothing = controls.smoothingMin;
		}
		setCxSmoothingSetting(gains.cXSmoothing);
		Serial.print("C-Stick X Smoothing decreased to:");
		Serial.println(gains.cXSmoothing);
	} else if(axis == YAXIS && increase == INCREASE) {
		gains.cYSmoothing = gains.cYSmoothing + 0.1;
		if (gains.cYSmoothing > controls.smoothingMax) {
			gains.cYSmoothing = controls.smoothingMax;
		}
		setCySmoothingSetting(gains.cYSmoothing);
		Serial.print("C-Stick Y Smoothing increased to:");
		Serial.println(gains.cYSmoothing);
	} else if(axis == YAXIS && increase == DECREASE) {
		gains.cYSmoothing = gains.cYSmoothing - 0.1;
		if (gains.cYSmoothing < controls.smoothingMin) {
			gains.cYSmoothing = controls.smoothingMin;
		}
		setCySmoothingSetting(gains.cYSmoothing);
		Serial.print("C-Stick Y Smoothing decreased to:");
		Serial.println(gains.cYSmoothing);
	}

	//recompute the intermediate gains used directly by the kalman filter
	recomputeGains(gains, normGains);

	btn.Cx = (uint8_t) (_floatOrigin + (gains.cXSmoothing * 10));
	btn.Cy = (uint8_t) (_floatOrigin + (gains.cYSmoothing * 10));

	clearButtons(2000, btn, hardware);
}

void adjustCstickOffset(const WhichAxis axis, const Increase increase, Buttons &btn, HardwareButtons &hardware, ControlConfig &controls) {
	Serial.println("Adjusting C-stick Offset");
	if(axis == XAXIS && increase == INCREASE) {
		controls.cXOffset++;
		if(controls.cXOffset > controls.cMax) {
			controls.cXOffset = controls.cMax;
		}
		setCXOffsetSetting(controls.cXOffset);
		Serial.print("X offset increased to:");
		Serial.println(controls.cXOffset);
	} else if(axis == XAXIS && increase == DECREASE) {
		controls.cXOffset--;
		if(controls.cXOffset < controls.cMin) {
			controls.cXOffset = controls.cMin;
		}
		setCXOffsetSetting(controls.cXOffset);
		Serial.print("X offset decreased to:");
		Serial.println(controls.cXOffset);
	} else if(axis == YAXIS && increase == INCREASE) {
		controls.cYOffset++;
		if(controls.cYOffset > controls.cMax) {
			controls.cYOffset = controls.cMax;
		}
		setCYOffsetSetting(controls.cYOffset);
		Serial.print("Y offset increased to:");
		Serial.println(controls.cYOffset);
	} else if(axis == YAXIS && increase == DECREASE) {
		controls.cYOffset--;
		if(controls.cYOffset < controls.cMin) {
			controls.cYOffset = controls.cMin;
		}
		setCYOffsetSetting(controls.cYOffset);
		Serial.print("Y offset decreased to:");
		Serial.println(controls.cYOffset);
	}

	btn.Cx = (uint8_t) (_floatOrigin + controls.cXOffset);
	btn.Cy = (uint8_t) (_floatOrigin + controls.cYOffset);

	clearButtons(2000, btn, hardware);
}

void showCstickSettings(Buttons &btn, HardwareButtons &hardware, ControlConfig &controls, FilterGains &gains) {
	//Snapback/smoothing on A-stick
	btn.Ax = (uint8_t) (_floatOrigin + (gains.cXSmoothing * 10));
	btn.Ay = (uint8_t) (_floatOrigin + (gains.cYSmoothing * 10));

	//Smoothing on C-stick
	btn.Cx = (uint8_t) (_floatOrigin + controls.cXOffset);
	btn.Cy = (uint8_t) (_floatOrigin + controls.cYOffset);

	clearButtons(2000, btn, hardware);
}

void adjustTriggerOffset(const WhichTrigger trigger, const Increase increase, Buttons &btn, HardwareButtons &hardware, ControlConfig &controls) {
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

	clearButtons(250, btn, hardware);
}

void setJump(ControlConfig &controls){
	switch(controls.jumpConfig){
			case SWAP_XZ:
				controls.pinZSwappable = _pinX;
				controls.pinXSwappable = _pinZ;
				controls.pinYSwappable = _pinY;
				break;
			case SWAP_YZ:
				controls.pinZSwappable = _pinY;
				controls.pinXSwappable = _pinX;
				controls.pinYSwappable = _pinZ;
				break;
			default:
				controls.pinZSwappable = _pinZ;
				controls.pinXSwappable = _pinX;
				controls.pinYSwappable = _pinY;
	}
}

void readJumpConfig(JumpConfig jumpConfig, ControlConfig &controls){
	Serial.print("setting jump to: ");
	if (controls.jumpConfig == jumpConfig) {
		controls.jumpConfig = DEFAULTJUMP;
		Serial.println("normal again");
	} else {
		controls.jumpConfig = jumpConfig;
		switch (jumpConfig) {
			case SWAP_XZ:
				Serial.println("X<->Z");
				break;
			case SWAP_YZ:
				Serial.println("Y<->Z");
				break;
			default:
				Serial.println("normal");
		}
	}
	setJumpSetting(controls.jumpConfig);
	setJump(controls);
}

void nextTriggerState(WhichTrigger trigger, Buttons &btn, HardwareButtons &hardware, ControlConfig &controls) {
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

	clearButtons(2000, btn, hardware);
}

void initializeButtons(Buttons &btn,int &startUpLa, int &startUpRa){
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
		startUpLa = max(startUpLa,adc->adc0->analogRead(_pinLa)>>4);
		startUpRa = max(startUpRa,adc->adc0->analogRead(_pinRa)>>4);
	}
	//set the trigger values to this measured startup value
	btn.La = startUpLa;
	btn.Ra = startUpRa;

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

#endif //PHOBGCC_H
