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
//#include "../rp2040/include/PicoProtoboard.h"    // For a protoboard with a Pico on it, used for developing for the RP2040
#include "../rp2040/include/Phob2_0.h"           // For PhobGCC Board 2.0 with RP2040

#include "structsAndEnums.h"
#include "variables.h"
#include "filter.h"
#include "stick.h"
#include "../extras/extras.h"

//#define BUILD_RELEASE
#define BUILD_DEV

//This is just an integer.
#define SW_VERSION 30

ControlConfig _controls{
	.aRemap = 1 << A_REMAP,
	.bRemap = 1 << B_REMAP,
	.lRemap = 1 << L_REMAP,
	.rRemap = 1 << R_REMAP,
	.xRemap = 1 << X_REMAP,
	.yRemap = 1 << Y_REMAP,
	.zRemap = 1 << Z_REMAP,
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
	.rumbleMin = -11,
	.rumbleMax = 11,
	.rumbleDefault = 9,//5 is the max for 3v cell rumble, 9 is for oem-feeling normal rumble motors
	.rumbleFactory = 9,
	.safeMode = true,
	.autoInit = false,
	.lTrigInitial = 0,
	.rTrigInitial = 0,
	.xSnapback = 4,
	.ySnapback = 4,
	.snapbackMin = -10,
	.snapbackMax = 10,
	.snapbackDefault = 4,
	.snapbackFactoryAX = 4,
	.snapbackFactoryAY = 4,
	.axSmoothing = 0,
	.aySmoothing = 0,
	.cxSmoothing = 5,
	.cySmoothing = 5,
	.smoothingMin = 0,
	.smoothingMax = 18,
	.snapbackFactoryCX = 5,
	.snapbackFactoryCY = 5,
	.smoothingFactoryAX = 0,
	.smoothingFactoryAY = 0,
	.axWaveshaping = 0,
	.ayWaveshaping = 0,
	.cxWaveshaping = 0,
	.cyWaveshaping = 0,
	.waveshapingMin = -24,
	.waveshapingMax = 24,
	.waveshapingDefault = 0,
	.waveshapingFactoryAX = 0,
	.waveshapingFactoryAY = 0,
	.waveshapingFactoryCX = 0,
	.waveshapingFactoryCY = 0,
	.astickCardinalSnapping = 6,
	.cstickCardinalSnapping = 6,
	.cardinalSnappingMin = -2,
	.cardinalSnappingMax = 6,
	.cardinalSnappingDefault = 6,
	.astickAnalogScaler = 100,
	.cstickAnalogScaler = 100,
	.analogScalerMin = 82,
	.analogScalerMax = 125,
	.analogScalerDefault = 100,
	.tournamentToggle = 0,
	.tournamentToggleMin = 0,
	.tournamentToggleMax = 5,
#ifdef PICO_RP2040
	.interlaceOffset = 0,
	.interlaceOffsetMin = -150,
	.interlaceOffsetMax = 150,
#endif //PICO_RP2040
};

FilterGains _gains {//these values are for 800 hz, recomputeGains converts them to what is needed for the actual frequency (1000 Hz)
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
	} else if(rumble < 0) {
		return -pow(2.0, 7+((-rumble-3)/8.0)); //should be 256 when rumble is 11
	} else {
		return 0;
	}
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
}

void changeRumble(const Increase increase, Buttons &btn, Buttons &hardware, ControlConfig &controls) {
	debug_println("changing rumble");
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
}

void adjustSnapback(const WhichAxis axis, const Increase increase, Buttons &btn, Buttons &hardware, ControlConfig &controls, FilterGains &gains, FilterGains &normGains){
	debug_println("adjusting snapback filtering");
	if(axis == XAXIS && increase == INCREASE){
		controls.xSnapback = min(controls.xSnapback+1, controls.snapbackMax);
		debug_print("x snapback filtering increased to:");
		debug_println(controls.xSnapback);
	}
	else if(axis == XAXIS && increase == DECREASE){
		controls.xSnapback = max(controls.xSnapback-1, controls.snapbackMin);
		debug_print("x snapback filtering decreased to:");
		debug_println(controls.xSnapback);
	}

	if(axis == YAXIS && increase == INCREASE){
		controls.ySnapback = min(controls.ySnapback+1, controls.snapbackMax);
		debug_print("y snapback filtering increased to:");
		debug_println(controls.ySnapback);
	}
	else if(axis == YAXIS && increase == DECREASE){
		controls.ySnapback = max(controls.ySnapback-1, controls.snapbackMin);
		debug_print("y snapback filtering decreased to:");
		debug_println(controls.ySnapback);
	}

    //recompute the intermediate gains used directly by the kalman filter
    recomputeGains(controls, gains, normGains);

	btn.Cx = (uint8_t) (controls.xSnapback + _floatOrigin);
	btn.Cy = (uint8_t) (controls.ySnapback + _floatOrigin);

	clearButtons(750, btn, hardware);

	setXSnapbackSetting(controls.xSnapback);
	setYSnapbackSetting(controls.ySnapback);
}

void adjustWaveshaping(const WhichStick whichStick, const WhichAxis axis, const Increase increase, Buttons &btn, Buttons &hardware, ControlConfig &controls){
	debug_println("adjusting waveshaping");
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

	clearButtons(750, btn, hardware);
}

void adjustSmoothing(const WhichAxis axis, const Increase increase, Buttons &btn, Buttons &hardware, ControlConfig &controls, FilterGains &gains, FilterGains &normGains) {
	debug_println("Adjusting Smoothing");
	if (axis == XAXIS && increase == INCREASE) {
		controls.axSmoothing++;
		if(controls.axSmoothing > controls.smoothingMax) {
			controls.axSmoothing = controls.smoothingMax;
		}
		setXSmoothingSetting(controls.axSmoothing);
		debug_print("X Smoothing increased to:");
		debug_println(controls.axSmoothing);
	} else if(axis == XAXIS && increase == DECREASE) {
		controls.axSmoothing--;
		if(controls.axSmoothing < controls.smoothingMin) {
			controls.axSmoothing = controls.smoothingMin;
		}
		setXSmoothingSetting(controls.axSmoothing);
		debug_print("X Smoothing decreased to:");
		debug_println(controls.axSmoothing);
	} else if(axis == YAXIS && increase == INCREASE) {
		controls.aySmoothing++;
		if (controls.aySmoothing > controls.smoothingMax) {
			controls.aySmoothing = controls.smoothingMax;
		}
		setYSmoothingSetting(controls.aySmoothing);
		debug_print("Y Smoothing increased to:");
		debug_println(controls.aySmoothing);
	} else if(axis == YAXIS && increase == DECREASE) {
		controls.aySmoothing--;
		if (controls.aySmoothing < controls.smoothingMin) {
			controls.aySmoothing = controls.smoothingMin;
		}
		setYSmoothingSetting(controls.aySmoothing);
		debug_print("Y Smoothing decreased to:");
		debug_println(controls.aySmoothing);
	}

	//recompute the intermediate gains used directly by the kalman filter
	recomputeGains(controls, gains, normGains);

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
	btn.La = (uint8_t) (controls.axWaveshaping >= 0) ? controls.axWaveshaping : 100-controls.axWaveshaping;
	btn.Ra = (uint8_t) (controls.ayWaveshaping >= 0) ? controls.ayWaveshaping : 100-controls.ayWaveshaping;

	clearButtons(2000, btn, hardware);
}

void adjustCstickSmoothing(const WhichAxis axis, const Increase increase, Buttons &btn, Buttons &hardware, ControlConfig &controls, FilterGains &gains, FilterGains &normGains) {
	debug_println("Adjusting C-Stick Smoothing");
	if (axis == XAXIS && increase == INCREASE) {
		controls.cxSmoothing++;
		if(controls.cxSmoothing > controls.smoothingMax) {
			controls.cxSmoothing = controls.smoothingMax;
		}
		setCxSmoothingSetting(controls.cxSmoothing);
		debug_print("C-Stick X Smoothing increased to:");
		debug_println(controls.cxSmoothing);
	} else if(axis == XAXIS && increase == DECREASE) {
		controls.cxSmoothing--;
		if(controls.cxSmoothing < controls.smoothingMin) {
			controls.cxSmoothing = controls.smoothingMin;
		}
		setCxSmoothingSetting(controls.cxSmoothing);
		debug_print("C-Stick X Smoothing decreased to:");
		debug_println(controls.cxSmoothing);
	} else if(axis == YAXIS && increase == INCREASE) {
		controls.cySmoothing++;
		if (controls.cySmoothing > controls.smoothingMax) {
			controls.cySmoothing = controls.smoothingMax;
		}
		setCySmoothingSetting(controls.cySmoothing);
		debug_print("C-Stick Y Smoothing increased to:");
		debug_println(controls.cySmoothing);
	} else if(axis == YAXIS && increase == DECREASE) {
		controls.cySmoothing--;
		if (controls.cySmoothing < controls.smoothingMin) {
			controls.cySmoothing = controls.smoothingMin;
		}
		setCySmoothingSetting(controls.cySmoothing);
		debug_print("C-Stick Y Smoothing decreased to:");
		debug_println(controls.cySmoothing);
	}

	//recompute the intermediate gains used directly by the kalman filter
	recomputeGains(controls, gains, normGains);

	btn.Cx = (uint8_t) (_floatOrigin + controls.cxSmoothing);
	btn.Cy = (uint8_t) (_floatOrigin + controls.cySmoothing);

	clearButtons(750, btn, hardware);
}

void adjustCstickOffset(const WhichAxis axis, const Increase increase, Buttons &btn, Buttons &hardware, ControlConfig &controls) {
	debug_println("Adjusting C-stick Offset");
	if(axis == XAXIS && increase == INCREASE) {
		controls.cXOffset++;
		if(controls.cXOffset > controls.cMax) {
			controls.cXOffset = controls.cMax;
		}
		setCxOffsetSetting(controls.cXOffset);
		debug_print("X offset increased to:");
		debug_println(controls.cXOffset);
	} else if(axis == XAXIS && increase == DECREASE) {
		controls.cXOffset--;
		if(controls.cXOffset < controls.cMin) {
			controls.cXOffset = controls.cMin;
		}
		setCxOffsetSetting(controls.cXOffset);
		debug_print("X offset decreased to:");
		debug_println(controls.cXOffset);
	} else if(axis == YAXIS && increase == INCREASE) {
		controls.cYOffset++;
		if(controls.cYOffset > controls.cMax) {
			controls.cYOffset = controls.cMax;
		}
		setCyOffsetSetting(controls.cYOffset);
		debug_print("Y offset increased to:");
		debug_println(controls.cYOffset);
	} else if(axis == YAXIS && increase == DECREASE) {
		controls.cYOffset--;
		if(controls.cYOffset < controls.cMin) {
			controls.cYOffset = controls.cMin;
		}
		setCyOffsetSetting(controls.cYOffset);
		debug_print("Y offset decreased to:");
		debug_println(controls.cYOffset);
	}

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
	btn.La = (uint8_t) (controls.cxWaveshaping >= 0) ? controls.cxWaveshaping : 100-controls.cxWaveshaping;
	btn.Ra = (uint8_t) (controls.cyWaveshaping >= 0) ? controls.cyWaveshaping : 100-controls.cyWaveshaping;

	clearButtons(2000, btn, hardware);
}

void adjustCardinalSnapping(const WhichStick whichStick, const Increase increase, Buttons &btn, Buttons &hardware, ControlConfig &controls) {
	if(whichStick == ASTICK && increase == INCREASE) {
		controls.astickCardinalSnapping++;
		if(controls.astickCardinalSnapping > controls.cardinalSnappingMax) {
			controls.astickCardinalSnapping = controls.cardinalSnappingMax;
		}
		setCardinalSnappingSetting(controls.astickCardinalSnapping, ASTICK);
	} else if(whichStick == ASTICK && increase == DECREASE) {
		controls.astickCardinalSnapping--;
		if(controls.astickCardinalSnapping < controls.cardinalSnappingMin) {
			controls.astickCardinalSnapping = controls.cardinalSnappingMin;
		}
		setCardinalSnappingSetting(controls.astickCardinalSnapping, ASTICK);
	} else if(whichStick == CSTICK && increase == INCREASE) {
		controls.cstickCardinalSnapping++;
		if(controls.cstickCardinalSnapping > controls.cardinalSnappingMax) {
			controls.cstickCardinalSnapping = controls.cardinalSnappingMax;
		}
		setCardinalSnappingSetting(controls.cstickCardinalSnapping, CSTICK);
	} else if(whichStick == CSTICK && increase == DECREASE) {
		controls.cstickCardinalSnapping--;
		if(controls.cstickCardinalSnapping < controls.cardinalSnappingMin) {
			controls.cstickCardinalSnapping = controls.cardinalSnappingMin;
		}
		setCardinalSnappingSetting(controls.cstickCardinalSnapping, CSTICK);
	}

	btn.Ax = (uint8_t) (_floatOrigin + controls.astickCardinalSnapping);
	btn.Ay = (uint8_t) (_floatOrigin + controls.astickCardinalSnapping);

	btn.Cx = (uint8_t) (_floatOrigin + controls.cstickCardinalSnapping);
	btn.Cy = (uint8_t) (_floatOrigin + controls.cstickCardinalSnapping);

	clearButtons(750, btn, hardware);

}

void adjustAnalogScaler(const WhichStick whichStick, const Increase increase, Buttons &btn, Buttons &hardware, ControlConfig &controls) {
	if(whichStick == ASTICK && increase == INCREASE) {
		controls.astickAnalogScaler++;
		if(controls.astickAnalogScaler > controls.analogScalerMax) {
			controls.astickAnalogScaler = controls.analogScalerMax;
		}
		setAnalogScalerSetting(controls.astickAnalogScaler, ASTICK);
	} else if(whichStick == ASTICK && increase == DECREASE) {
		controls.astickAnalogScaler--;
		if(controls.astickAnalogScaler < controls.analogScalerMin) {
			controls.astickAnalogScaler = controls.analogScalerMin;
		}
		setAnalogScalerSetting(controls.astickAnalogScaler, ASTICK);
	} else if(whichStick == CSTICK && increase == INCREASE) {
		controls.cstickAnalogScaler++;
		if(controls.cstickAnalogScaler > controls.analogScalerMax) {
			controls.cstickAnalogScaler = controls.analogScalerMax;
		}
		setAnalogScalerSetting(controls.cstickAnalogScaler, CSTICK);
	} else if(whichStick == CSTICK && increase == DECREASE) {
		controls.cstickAnalogScaler--;
		if(controls.cstickAnalogScaler < controls.analogScalerMin) {
			controls.cstickAnalogScaler = controls.analogScalerMin;
		}
		setAnalogScalerSetting(controls.cstickAnalogScaler, CSTICK);
	}

	btn.Ax = (uint8_t) (_floatOrigin + controls.astickAnalogScaler);
	btn.Ay = (uint8_t) (_floatOrigin + controls.astickAnalogScaler);

	btn.Cx = (uint8_t) (_floatOrigin + controls.cstickAnalogScaler);
	btn.Cy = (uint8_t) (_floatOrigin + controls.cstickAnalogScaler);

	clearButtons(750, btn, hardware);
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

void changeTournamentToggle(Buttons &btn, Buttons &hardware, ControlConfig &controls) {
	if(controls.tournamentToggle == controls.tournamentToggleMax) {
		controls.tournamentToggle = 0;
	} else {
		controls.tournamentToggle++;
	}

	setTournamentToggleSetting(controls.tournamentToggle);

	btn.Ax = (uint8_t) (_floatOrigin);
	btn.Ay = (uint8_t) (_floatOrigin);

	btn.Cx = (uint8_t) (_floatOrigin);
	btn.Cy = (uint8_t) (_floatOrigin + controls.tournamentToggle+1);

	clearButtons(750, btn, hardware);
}

//apply digital button swaps for ABLRXYZ
void applyRemaps(const ControlConfig &controls, const Buttons &hardware, Buttons &btn){
	uint8_t source =
		hardware.A << A_REMAP |
		hardware.B << B_REMAP |
		hardware.L << L_REMAP |
		hardware.R << R_REMAP |
		hardware.X << X_REMAP |
		hardware.Y << Y_REMAP |
		hardware.Z << Z_REMAP;
	btn.A = (source & controls.aRemap) != 0;
	btn.B = (source & controls.bRemap) != 0;
	btn.L = (source & controls.lRemap) != 0;
	btn.R = (source & controls.rRemap) != 0;
	btn.X = (source & controls.xRemap) != 0;
	btn.Y = (source & controls.yRemap) != 0;
	btn.Z = (source & controls.zRemap) != 0;
}

void remapAdvance(int &step, ControlConfig &controls, Buttons &hardware, Buttons &btn) {
	if(step == -1) {
		//clear the mappings
		controls.aRemap = 0;
		controls.bRemap = 0;
		controls.lRemap = 0;
		controls.rRemap = 0;
		controls.xRemap = 0;
		controls.yRemap = 0;
		controls.zRemap = 0;
		step = 0;
		return;
	}
	const uint8_t source =
		(hardware.A ? 1 : 0) << A_REMAP |
		(hardware.B ? 1 : 0) << B_REMAP |
		(hardware.L ? 1 : 0) << L_REMAP |
		(hardware.R ? 1 : 0) << R_REMAP |
		(hardware.X ? 1 : 0) << X_REMAP |
		(hardware.Y ? 1 : 0) << Y_REMAP |
		(hardware.Z ? 1 : 0) << Z_REMAP;
	const uint8_t mask = controls.aRemap |
		controls.bRemap |
		controls.lRemap |
		controls.rRemap |
		controls.xRemap |
		controls.yRemap |
		controls.zRemap;
	if ((mask & source) != 0) {
		//a button that's already been mapped is currently pressed, so do nothing
		return;
	}
	const int pressCount =
		hardware.A +
		hardware.B +
		hardware.L +
		hardware.R +
		hardware.X +
		hardware.Y +
		hardware.Z;
	uint8_t pressedButton = 0;
	if(pressCount == 1) {
		if(hardware.A) {pressedButton = 1 << A_REMAP;}
		if(hardware.B) {pressedButton = 1 << B_REMAP;}
		if(hardware.L) {pressedButton = 1 << L_REMAP;}
		if(hardware.R) {pressedButton = 1 << R_REMAP;}
		if(hardware.X) {pressedButton = 1 << X_REMAP;}
		if(hardware.Y) {pressedButton = 1 << Y_REMAP;}
		if(hardware.Z) {pressedButton = 1 << Z_REMAP;}
	} else {
		//if no buttons or pressed, or somehow two buttons are pressed simultaneously,
		//do nothing
		return;
	}
	//if we got to this point, exactly one button is pressed and it's new
	switch(step) {
		case 0:
			controls.aRemap = pressedButton;
			break;
		case 1:
			controls.bRemap = pressedButton;
			break;
		case 2:
			controls.lRemap = pressedButton;
			break;
		case 3:
			controls.rRemap = pressedButton;
			break;
		case 4:
			controls.xRemap = pressedButton;
			break;
		case 5:
			controls.yRemap = pressedButton;
			break;
		case 6:
			controls.zRemap = pressedButton;
			break;
		default:
			break;
	}
	step++;
	if(step > Z_REMAP) {
		step = -1;
		setRemapSetting(controls.aRemap,
				controls.bRemap,
				controls.lRemap,
				controls.rRemap,
				controls.xRemap,
				controls.yRemap,
				controls.zRemap);
#ifdef BATCHSETTINGS
		commitSettings();
#endif //BATCHSETTINGS
		freezeSticks(2000, btn, hardware);
	}
}

void resetRemap(ControlConfig &controls) {
	//clear the mappings
	controls.aRemap = 1 << A_REMAP;
	controls.bRemap = 1 << B_REMAP;
	controls.lRemap = 1 << L_REMAP;
	controls.rRemap = 1 << R_REMAP;
	controls.xRemap = 1 << X_REMAP;
	controls.yRemap = 1 << Y_REMAP;
	controls.zRemap = 1 << Z_REMAP;
	setRemapSetting(controls.aRemap,
			controls.bRemap,
			controls.lRemap,
			controls.rRemap,
			controls.xRemap,
			controls.yRemap,
			controls.zRemap);
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

	//We want to one-index the modes for the users, so we add 1 here
	btn.Ax = (uint8_t) (_floatOrigin + controls.lConfig + 1);
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
void applyCalFromPoints(const WhichStick whichStick, float notchAngles[], const float tempCalPointsX[], const float tempCalPointsY[], NotchStatus notchStatus[], float measuredNotchAngles[], StickParams &stickParams, const ControlConfig &controls) {
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
	transformCalPoints(cleanedPointsX, cleanedPointsY, transformedX, transformedY, stickParams, controls, whichStick);
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

	//get the remaps
	getRemapSetting(controls.aRemap,
			controls.bRemap,
			controls.lRemap,
			controls.rRemap,
			controls.xRemap,
			controls.yRemap,
			controls.zRemap);
	if((controls.aRemap ^ controls.bRemap ^ controls.lRemap ^ controls.rRemap ^ controls.xRemap ^ controls.yRemap ^ controls.zRemap) !=
			127) {
		controls.aRemap = 1 << A_REMAP;
		controls.bRemap = 1 << B_REMAP;
		controls.lRemap = 1 << L_REMAP;
		controls.rRemap = 1 << R_REMAP;
		controls.xRemap = 1 << X_REMAP;
		controls.yRemap = 1 << Y_REMAP;
		controls.zRemap = 1 << Z_REMAP;
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
	debug_print("the xSnapback value from eeprom is:");
	debug_println(controls.xSnapback);
	if(controls.xSnapback < controls.snapbackMin) {
		controls.xSnapback = controls.snapbackDefault;
		numberOfNaN++;
	} else if (controls.xSnapback > controls.snapbackMax) {
		controls.xSnapback = controls.snapbackDefault;
		numberOfNaN++;
	}

	//get the y-ayis snapback correction
	controls.ySnapback = getYSnapbackSetting();
	debug_print("the ySnapback value from eeprom is:");
	debug_println(controls.ySnapback);
	if(controls.ySnapback < controls.snapbackMin) {
		controls.ySnapback = controls.snapbackDefault;
		numberOfNaN++;
	} else if (controls.ySnapback > controls.snapbackMax) {
		controls.ySnapback = controls.snapbackDefault;
		numberOfNaN++;
	}

	//get the x-axis smoothing value
	controls.axSmoothing = getXSmoothingSetting();
	debug_print("the xSmoothing value from eeprom is:");
	debug_println(controls.axSmoothing);
	if(controls.axSmoothing > controls.smoothingMax) {
		controls.axSmoothing = controls.smoothingMin;
		numberOfNaN++;
		debug_print("the xSmoothing value was adjusted to:");
		debug_println(controls.axSmoothing);
	} else if(controls.axSmoothing < controls.smoothingMin) {
		controls.axSmoothing = controls.smoothingMin;
		numberOfNaN++;
		debug_print("the xSmoothing value was adjusted to:");
		debug_println(controls.axSmoothing);
	}

	//get the y-axis smoothing value
	controls.aySmoothing = getYSmoothingSetting();
	debug_print("the ySmoothing value from eeprom is:");
	debug_println(controls.aySmoothing);
	if(controls.aySmoothing > controls.smoothingMax) {
		controls.aySmoothing = controls.smoothingMin;
		numberOfNaN++;
		debug_print("the ySmoothing value was adjusted to:");
		debug_println(controls.aySmoothing);
	} else if(controls.aySmoothing < controls.smoothingMin) {
		controls.aySmoothing = controls.smoothingMin;
		numberOfNaN++;
		debug_print("the ySmoothing value was adjusted to:");
		debug_println(controls.aySmoothing);
	}

	//get the c-stick x-axis smoothing value
	controls.cxSmoothing = getCxSmoothingSetting();
	debug_print("the cXSmoothing value from eeprom is:");
	debug_println(controls.cxSmoothing);
	if(controls.cxSmoothing > controls.smoothingMax) {
		controls.cxSmoothing = controls.smoothingMin;
		numberOfNaN++;
		debug_print("the cXSmoothing value was adjusted to:");
		debug_println(controls.cxSmoothing);
	} else if(controls.cxSmoothing < controls.smoothingMin) {
		controls.cxSmoothing = controls.smoothingMin;
		numberOfNaN++;
		debug_print("the cXSmoothing value was adjusted to:");
		debug_println(controls.cxSmoothing);
	}

	//get the c-stick y-axis smoothing value
	controls.cySmoothing = getCySmoothingSetting();
	debug_print("the cYSmoothing value from eeprom is:");
	debug_println(controls.cySmoothing);
	if(controls.cySmoothing > controls.smoothingMax) {
		controls.cySmoothing = controls.smoothingMin;
		numberOfNaN++;
		debug_print("the cYSmoothing value was adjusted to:");
		debug_println(controls.cySmoothing);
	} else if(controls.cySmoothing < controls.smoothingMin) {
		controls.cySmoothing = controls.smoothingMin;
		numberOfNaN++;
		debug_print("the cYSmoothing value was adjusted to:");
		debug_println(controls.cySmoothing);
	}

	//get the a-stick x-axis waveshaping value
	controls.axWaveshaping = getWaveshapingSetting(ASTICK, XAXIS);
	debug_print("the axWaveshaping value from eeprom is:");
	debug_println(controls.axWaveshaping);
	if(controls.axWaveshaping < controls.waveshapingMin) {
		controls.axWaveshaping = controls.waveshapingDefault;
		numberOfNaN++;
	} else if (controls.axWaveshaping > controls.waveshapingMax) {
		controls.axWaveshaping = controls.waveshapingDefault;
		numberOfNaN++;
	}

	//get the a-stick y-axis waveshaping value
	controls.ayWaveshaping = getWaveshapingSetting(ASTICK, YAXIS);
	debug_print("the ayWaveshaping value from eeprom is:");
	debug_println(controls.ayWaveshaping);
	if(controls.ayWaveshaping < controls.waveshapingMin) {
		controls.ayWaveshaping = controls.waveshapingDefault;
		numberOfNaN++;
	} else if (controls.ayWaveshaping > controls.waveshapingMax) {
		controls.ayWaveshaping = controls.waveshapingDefault;
		numberOfNaN++;
	}

	//get the c-stick x-axis waveshaping value
	controls.cxWaveshaping = getWaveshapingSetting(CSTICK, XAXIS);
	debug_print("the cxWaveshaping value from eeprom is:");
	debug_println(controls.cxWaveshaping);
	if(controls.cxWaveshaping < controls.waveshapingMin) {
		controls.cxWaveshaping = controls.waveshapingDefault;
		numberOfNaN++;
	} else if (controls.cxWaveshaping > controls.waveshapingMax) {
		controls.cxWaveshaping = controls.waveshapingDefault;
		numberOfNaN++;
	}

	//get the c-stick y-axis waveshaping value
	controls.cyWaveshaping = getWaveshapingSetting(CSTICK, YAXIS);
	debug_print("the cyWaveshaping value from eeprom is:");
	debug_println(controls.cyWaveshaping);
	if(controls.cyWaveshaping < controls.waveshapingMin) {
		controls.cyWaveshaping = controls.waveshapingDefault;
		numberOfNaN++;
	} else if (controls.cyWaveshaping > controls.waveshapingMax) {
		controls.cyWaveshaping = controls.waveshapingDefault;
		numberOfNaN++;
	}

	if(controls.axWaveshaping != 0){
		debug_print("the axWaveshaping coefficient is: ");
		debug_println(1/calcWaveshapeMult(controls.axWaveshaping));
	}

	//recompute the intermediate gains used directly by the kalman filter
	recomputeGains(controls, gains, normGains);

	//Get the rumble value
	controls.rumble = getRumbleSetting();
	debug_print("Rumble value before fixing: ");
	debug_println(controls.rumble);
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
	debug_print("Rumble value: ");
	debug_println(controls.rumble);
	debug_print("Rumble power: ");
	debug_println(_rumblePower);

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
	debug_print("Auto init: ");
	debug_println(controls.autoInit);

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
	debug_println("calibration points cleaned");
	linearizeCal(cleanedPointsX, cleanedPointsY, cleanedPointsX, cleanedPointsY, aStickParams);
	debug_println("A stick linearized");
	notchCalibrate(cleanedPointsX, cleanedPointsY, notchPointsX, notchPointsY, _noOfNotches, aStickParams);

	//get the calibration points collected during the last A stick calibration
	getPointsSetting(tempCalPointsX, CSTICK, XAXIS);
	getPointsSetting(tempCalPointsY, CSTICK, YAXIS);
	getNotchAnglesSetting(notchAngles, CSTICK);
	cleanCalPoints(tempCalPointsX, tempCalPointsY, notchAngles, cleanedPointsX, cleanedPointsY, notchPointsX, notchPointsY, notchStatus);
	debug_println("calibration points cleaned");
	linearizeCal(cleanedPointsX, cleanedPointsY, cleanedPointsX, cleanedPointsY, cStickParams);
	debug_println("C stick linearized");
	notchCalibrate(cleanedPointsX, cleanedPointsY, notchPointsX, notchPointsY, _noOfNotches, cStickParams);

	//read in extras settings
	for(int extra=0; extra<EXTRAS_SIZE; extra++){
		ExtrasSlot slot = (ExtrasSlot)extra;
		for(int offset=0; offset<4; offset++){
			controls.extras[slot].config[offset].intValue = getExtrasSettingInt(slot, offset);
		}
	}

	//get the A-stick cardinal snapping
	controls.astickCardinalSnapping = getCardinalSnappingSetting(ASTICK);
	if(controls.astickCardinalSnapping > controls.cardinalSnappingMax) {
		controls.astickCardinalSnapping = controls.cardinalSnappingDefault;
		numberOfNaN++;
	} else if (controls.astickCardinalSnapping < controls.cardinalSnappingMin) {
		controls.astickCardinalSnapping = controls.cardinalSnappingDefault;
		numberOfNaN++;
	}

	//get the C-stick cardinal snapping
	controls.cstickCardinalSnapping = getCardinalSnappingSetting(CSTICK);
	if(controls.cstickCardinalSnapping > controls.cardinalSnappingMax) {
		controls.cstickCardinalSnapping = controls.cardinalSnappingDefault;
		numberOfNaN++;
	} else if (controls.cstickCardinalSnapping < controls.cardinalSnappingMin) {
		controls.cstickCardinalSnapping = controls.cardinalSnappingDefault;
		numberOfNaN++;
	}

	//get the A-stick analog scaler
	controls.astickAnalogScaler = getAnalogScalerSetting(ASTICK);
	if(controls.astickAnalogScaler > controls.analogScalerMax) {
		controls.astickAnalogScaler = controls.analogScalerDefault;
		numberOfNaN++;
	} else if (controls.astickAnalogScaler < controls.analogScalerMin) {
		controls.astickAnalogScaler = controls.analogScalerDefault;
		numberOfNaN++;
	}

	//get the C-stick analog scaler
	controls.cstickAnalogScaler = getAnalogScalerSetting(CSTICK);
	if(controls.cstickAnalogScaler > controls.analogScalerMax) {
		controls.cstickAnalogScaler = controls.analogScalerDefault;
		numberOfNaN++;
	} else if (controls.cstickAnalogScaler < controls.analogScalerMin) {
		controls.cstickAnalogScaler = controls.analogScalerDefault;
		numberOfNaN++;
	}

	//get the tournament toggle setting
	controls.tournamentToggle = getTournamentToggleSetting();
	if(controls.tournamentToggle > controls.tournamentToggleMax) {
		controls.tournamentToggle = 0;
		numberOfNaN++;
	} else if(controls.tournamentToggle < controls.tournamentToggleMin) {
		controls.tournamentToggle = 0;
		numberOfNaN++;
	}

#ifdef PICO_RP2040
	_controls.interlaceOffset = getInterlaceOffsetSetting();
	if(controls.interlaceOffset < controls.interlaceOffsetMin) {
		controls.interlaceOffset = 0;
		numberOfNaN++;
	}
	if(controls.interlaceOffset > controls.interlaceOffsetMax) {
		controls.interlaceOffset = 0;
		numberOfNaN++;
	}
#endif //PICO_RP2040

	//Migration
	const int schema = getSchemaSetting();
	debug_print("Saved settings schema: ");
	debug_println(schema);
	bool migrating = false;
	switch(schema) {
		case -1:
			migrating = true;
			debug_println("Updating settings from unitialized");

			//write to controls AND write to settings
			controls.rumble = controls.rumble!=0 ? controls.rumble + 4 : 0;
			_rumblePower = calcRumblePower(controls.rumble);
			setRumbleSetting(controls.rumble);
			debug_print("Rumble value changed to: ");
			debug_println(controls.rumble);
			debug_print("Rumble power now: ");
			debug_println(_rumblePower);
			//fallthrough
		case 28:
			migrating = true;
			debug_println("Updating settings from 0.28");

			//write to controls AND write to settings
#ifdef PICO_RP2040
			controls.interlaceOffset = 0;
			setInterlaceOffsetSetting(0);
#endif //PICO_RP2040
			controls.astickAnalogScaler = controls.analogScalerDefault;
			setAnalogScalerSetting(controls.analogScalerDefault, ASTICK);
			controls.cstickAnalogScaler = controls.analogScalerDefault;
			setAnalogScalerSetting(controls.analogScalerDefault, CSTICK);
			controls.astickCardinalSnapping = controls.cardinalSnappingDefault;
			setCardinalSnappingSetting(controls.cardinalSnappingDefault, ASTICK);
			controls.cstickCardinalSnapping = controls.cardinalSnappingDefault;
			setCardinalSnappingSetting(controls.cardinalSnappingDefault, CSTICK);
			controls.tournamentToggle = controls.tournamentToggleMin;
			setTournamentToggleSetting(controls.tournamentToggleMin);
			//fallthrough
			/*
		case 29:
			//uncomment these when we do have it migrate
			migrating = true;
			debug_println("Updating settings from 0.29");

			//write to controls AND write to settings!!

			//fix the cardinal snapping just in case
			controls.astickCardinalSnapping = controls.cardinalSnappingDefault;
			setCardinalSnappingSetting(controls.cardinalSnappingDefault, ASTICK);
			controls.cstickCardinalSnapping = controls.cardinalSnappingDefault;
			setCardinalSnappingSetting(controls.cardinalSnappingDefault, CSTICK);

			if(controls.axSmoothing == 9) {
				controls.axSmoothing = 18;
				setXSmoothingSetting(18);
			}
			if(controls.aySmoothing == 9) {
				controls.aySmoothing = 18;
				setYSmoothingSetting(18);
			}
			if(controls.cxSmoothing == 9) {
				controls.cxSmoothing = 18;
				setCxSmoothingSetting(18);
			}
			if(controls.cySmoothing == 9) {
				controls.cySmoothing = 18;
				setCySmoothingSetting(18);
			}

			//set remaps to defaults
			controls.aRemap = 1 << A_REMAP;
			controls.bRemap = 1 << B_REMAP;
			controls.lRemap = 1 << L_REMAP;
			controls.rRemap = 1 << R_REMAP;
			controls.xRemap = 1 << X_REMAP;
			controls.yRemap = 1 << Y_REMAP;
			controls.zRemap = 1 << Z_REMAP;
			setRemapSetting(controls.aRemap, controls.bRemap, controls.lRemap, controls.rRemap, controls.xRemap, controls.yRemap, controls.zRemap);

			//fallthrough
			*/
			/*
		case 30:
			//uncomment these when we do have it migrate
			migrating = true;
			debug_println("Updating settings from 0.30");

			//write to controls AND write to settings!!
			//fallthrough
			*/
		default:
			if(migrating) {
				debug_println("Updating saved settings schema");
				setSchemaSetting(SW_VERSION);
#ifdef BATCHSETTINGS
				commitSettings(noLock);
#endif //BATCHSETTINGS
			}
	}
	return numberOfNaN;
}

void resetDefaults(HardReset reset, ControlConfig &controls, FilterGains &gains, FilterGains &normGains, StickParams &aStickParams, StickParams &cStickParams, const bool noLock = false){
	debug_println("RESETTING ALL DEFAULTS");

	controls.aRemap = 1 << A_REMAP;
	controls.bRemap = 1 << B_REMAP;
	controls.lRemap = 1 << L_REMAP;
	controls.rRemap = 1 << R_REMAP;
	controls.xRemap = 1 << X_REMAP;
	controls.yRemap = 1 << Y_REMAP;
	controls.zRemap = 1 << Z_REMAP;
	setRemapSetting(controls.aRemap, controls.bRemap, controls.lRemap, controls.rRemap, controls.xRemap, controls.yRemap, controls.zRemap);

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
	setYSnapbackSetting(controls.ySnapback);

	if(reset == FACTORY){
		controls.axSmoothing = controls.smoothingFactoryAX;
		controls.aySmoothing = controls.smoothingFactoryAY;
		controls.cxSmoothing = controls.snapbackFactoryCX;
		controls.cySmoothing = controls.snapbackFactoryCY;
	} else {
		controls.axSmoothing = controls.smoothingMin;
		controls.aySmoothing = controls.smoothingMin;
		controls.cxSmoothing = controls.snapbackFactoryCX;
		controls.cySmoothing = controls.snapbackFactoryCY;
	}
	setXSmoothingSetting(controls.axSmoothing);
	setYSmoothingSetting(controls.aySmoothing);
	setCxSmoothingSetting(controls.cxSmoothing);
	setCySmoothingSetting(controls.cySmoothing);
	//recompute the intermediate gains used directly by the kalman filter
	recomputeGains(controls, gains, normGains);

	if(reset == FACTORY){
		controls.axWaveshaping = controls.waveshapingFactoryAX;
		controls.ayWaveshaping = controls.waveshapingFactoryAY;
		controls.cxWaveshaping = controls.waveshapingFactoryCX;
		controls.cyWaveshaping = controls.waveshapingFactoryCY;
	} else {
		controls.axWaveshaping = controls.waveshapingDefault;
		controls.ayWaveshaping = controls.waveshapingDefault;
		controls.cxWaveshaping = controls.waveshapingDefault;
		controls.cyWaveshaping = controls.waveshapingDefault;
	}
	setWaveshapingSetting(controls.waveshapingDefault, ASTICK, XAXIS);
	setWaveshapingSetting(controls.waveshapingDefault, ASTICK, YAXIS);
	setWaveshapingSetting(controls.waveshapingDefault, CSTICK, XAXIS);
	setWaveshapingSetting(controls.waveshapingDefault, CSTICK, YAXIS);

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

	//Cardinal snapping
	controls.astickCardinalSnapping = controls.cardinalSnappingDefault;
	controls.cstickCardinalSnapping = controls.cardinalSnappingDefault;
	setCardinalSnappingSetting(controls.astickCardinalSnapping, ASTICK);
	setCardinalSnappingSetting(controls.cstickCardinalSnapping, CSTICK);

	//Analog scaling
	controls.astickAnalogScaler = controls.analogScalerDefault;
	controls.cstickAnalogScaler = controls.analogScalerDefault;
	setAnalogScalerSetting(controls.astickAnalogScaler, ASTICK);
	setAnalogScalerSetting(controls.cstickAnalogScaler, CSTICK);

#ifdef PICO_RP2040
	_controls.interlaceOffset = 0;
	setInterlaceOffsetSetting(0);
#endif //PICO_RP2040

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

		setPointsSetting(_defaultCalPoints, ASTICK, XAXIS);
		setPointsSetting(_defaultCalPoints, ASTICK, YAXIS);
		debug_println("A calibration points stored in EEPROM");

		float cleanedPointsX[_noOfNotches+1];
		float cleanedPointsY[_noOfNotches+1];
		float notchPointsX[_noOfNotches+1];
		float notchPointsY[_noOfNotches+1];
		NotchStatus notchStatus[_noOfNotches];

		cleanCalPoints(_defaultCalPoints, _defaultCalPoints, notchAngles, cleanedPointsX, cleanedPointsY, notchPointsX, notchPointsY, notchStatus);
		debug_println("A calibration points cleaned");
		linearizeCal(cleanedPointsX, cleanedPointsY, cleanedPointsX, cleanedPointsY, aStickParams);
		debug_println("A stick linearized");
		notchCalibrate(cleanedPointsX, cleanedPointsY, notchPointsX, notchPointsY, _noOfNotches, aStickParams);

		setPointsSetting(_defaultCalPoints, CSTICK, XAXIS);
		setPointsSetting(_defaultCalPoints, CSTICK, YAXIS);
		debug_println("C calibration points stored in EEPROM");

		cleanCalPoints(_defaultCalPoints, _defaultCalPoints, notchAngles, cleanedPointsX, cleanedPointsY, notchPointsX, notchPointsY, notchStatus);
		debug_println("C calibration points cleaned");
		linearizeCal(cleanedPointsX, cleanedPointsY, cleanedPointsX, cleanedPointsY, cStickParams);
		debug_println("C stick linearized");
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

void calibrationSkipMeasurement(int &currentCalStep, const WhichStick whichStick, float tempCalPointsX[], float tempCalPointsY[], NotchStatus notchStatus[], float notchAngles[], float measuredNotchAngles[], StickParams &aStickParams, StickParams &cStickParams, const ControlConfig &controls) {
	currentCalStep = _noOfCalibrationPoints;
	//Do the same thing we would have done at step 32 had we actually collected the points, but with stored tempCalPoints
	if(whichStick == CSTICK){
		//get the calibration points collected during the last stick calibration
		getPointsSetting(tempCalPointsX, whichStick, XAXIS);
		getPointsSetting(tempCalPointsY, whichStick, YAXIS);
		applyCalFromPoints(whichStick, notchAngles, tempCalPointsX, tempCalPointsY, notchStatus, measuredNotchAngles, cStickParams, controls);
	} else if(whichStick == ASTICK){
		//get the calibration points collected during the last stick calibration
		getPointsSetting(tempCalPointsX, whichStick, XAXIS);
		getPointsSetting(tempCalPointsY, whichStick, YAXIS);
		applyCalFromPoints(whichStick, notchAngles, tempCalPointsX, tempCalPointsY, notchStatus, measuredNotchAngles, aStickParams, controls);
	}
}

void calibrationUndo(int &currentCalStep, const WhichStick whichStick, NotchStatus notchStatus[]) {
	if(currentCalStep % 2 == 0 && currentCalStep < 32 && currentCalStep != 0 ) {
		//If it's measuring zero, go back to the previous zero
		currentCalStep -= 2;
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
}

void calibrationAdvance(ControlConfig &controls, int &currentCalStep, const WhichStick whichStick, float tempCalPointsX[], float tempCalPointsY[], bool &undoCal, float notchAngles[], NotchStatus notchStatus[], float measuredNotchAngles[], StickParams &aStickParams, StickParams &cStickParams) {
	if(whichStick == CSTICK){
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
			//bring all notches into a legal range; this helps recover from freakout situations
			legalizeNotches(currentCalStep, measuredNotchAngles, notchAngles, notchStatus);
			applyCalFromPoints(whichStick, notchAngles, tempCalPointsX, tempCalPointsY, notchStatus, measuredNotchAngles, cStickParams, controls);
		}
		int notchIndex = _notchAdjOrder[min(currentCalStep-_noOfCalibrationPoints, _noOfAdjNotches-1)];//limit this so it doesn't access outside the array bounds
		while((currentCalStep >= _noOfCalibrationPoints) && (notchStatus[notchIndex] == TERT_INACTIVE) && (currentCalStep < _noOfCalibrationPoints + _noOfAdjNotches)){//this non-diagonal notch was not calibrated
			legalizeNotches(currentCalStep, measuredNotchAngles, notchAngles, notchStatus);
			//skip to the next valid notch
			currentCalStep++;
			notchIndex = _notchAdjOrder[min(currentCalStep-_noOfCalibrationPoints, _noOfAdjNotches-1)];//limit this so it doesn't access outside the array bounds
		}
		if(currentCalStep >= _noOfCalibrationPoints + _noOfAdjNotches){//done adjusting notches
			debug_println("finished adjusting notches for the C stick");
			setPointsSetting(tempCalPointsX, whichStick, XAXIS);
			setPointsSetting(tempCalPointsY, whichStick, YAXIS);
			setNotchAnglesSetting(notchAngles, whichStick);
			controls.autoInit = 0;
			setAutoInitSetting(controls.autoInit);
#ifdef BATCHSETTINGS
			commitSettings();
#endif //BATCHSETTINGS
			debug_println("calibration points stored in EEPROM");
			float cleanedPointsX[_noOfNotches+1];
			float cleanedPointsY[_noOfNotches+1];
			float notchPointsX[_noOfNotches+1];
			float notchPointsY[_noOfNotches+1];
			cleanCalPoints(tempCalPointsX, tempCalPointsY, notchAngles, cleanedPointsX, cleanedPointsY, notchPointsX, notchPointsY, notchStatus);
			debug_println("calibration points cleaned");
			linearizeCal(cleanedPointsX, cleanedPointsY, cleanedPointsX, cleanedPointsY, cStickParams);
			debug_println("C stick linearized");
			notchCalibrate(cleanedPointsX, cleanedPointsY, notchPointsX, notchPointsY, _noOfNotches, cStickParams);
			currentCalStep = -1;
		}
	}
	else if (whichStick == ASTICK){
		debug_println("Current step:");
		debug_println(currentCalStep);
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
			//bring all notches into a legal range; this helps recover from freakout situations
			legalizeNotches(currentCalStep, measuredNotchAngles, notchAngles, notchStatus);
			applyCalFromPoints(whichStick, notchAngles, tempCalPointsX, tempCalPointsY, notchStatus, measuredNotchAngles, aStickParams, controls);
		}
		int notchIndex = _notchAdjOrder[min(currentCalStep-_noOfCalibrationPoints, _noOfAdjNotches-1)];//limit this so it doesn't access outside the array bounds
		while((currentCalStep >= _noOfCalibrationPoints) && (notchStatus[notchIndex] == TERT_INACTIVE) && (currentCalStep < _noOfCalibrationPoints + _noOfAdjNotches)){//this non-diagonal notch was not calibrated
			legalizeNotches(currentCalStep, measuredNotchAngles, notchAngles, notchStatus);
			//skip to the next valid notch
			currentCalStep++;
			notchIndex = _notchAdjOrder[min(currentCalStep-_noOfCalibrationPoints, _noOfAdjNotches-1)];//limit this so it doesn't access outside the array bounds
		}
		if(currentCalStep >= _noOfCalibrationPoints + _noOfAdjNotches){//done adjusting notches
			debug_println("finished adjusting notches for the A stick");
			setPointsSetting(tempCalPointsX, whichStick, XAXIS);
			setPointsSetting(tempCalPointsY, whichStick, YAXIS);
			setNotchAnglesSetting(notchAngles, whichStick);
			controls.autoInit = 0;
			setAutoInitSetting(controls.autoInit);
#ifdef BATCHSETTINGS
			commitSettings();
#endif //BATCHSETTINGS
			debug_println("calibration points stored in EEPROM");
			float cleanedPointsX[_noOfNotches+1];
			float cleanedPointsY[_noOfNotches+1];
			float notchPointsX[_noOfNotches+1];
			float notchPointsY[_noOfNotches+1];
			cleanCalPoints(tempCalPointsX, tempCalPointsY, notchAngles, cleanedPointsX, cleanedPointsY, notchPointsX, notchPointsY, notchStatus);
			debug_println("calibration points cleaned");
			linearizeCal(cleanedPointsX, cleanedPointsY, cleanedPointsX, cleanedPointsY, aStickParams);
			debug_println("A stick linearized");
			notchCalibrate(cleanedPointsX, cleanedPointsY, notchPointsX, notchPointsY, _noOfNotches, aStickParams);
			currentCalStep = -1;
		}
	}
}

void processButtons(Pins &pin, Buttons &btn, Buttons &hardware, ControlConfig &controls, FilterGains &gains, FilterGains &normGains, int &currentCalStep, int &currentRemapStep, bool &currentlyRaw, bool &running, float tempCalPointsX[], float tempCalPointsY[], WhichStick &whichStick, NotchStatus notchStatus[], float notchAngles[], float measuredNotchAngles[], StickParams &aStickParams, StickParams &cStickParams){
	//Gather the button data from the hardware
	readButtons(pin, hardware);
	hardware.La = (uint8_t) readLa(pin, controls.lTrigInitial, 1);
	hardware.Ra = (uint8_t) readRa(pin, controls.rTrigInitial, 1);

	//Copy hardware buttons into a temp
	Buttons tempBtn;
	copyButtons(hardware, tempBtn);

	//Implement button remaps
	applyRemaps(controls, hardware, tempBtn);

	//read the L and R sliders here instead of readSticks so we don't get race conditions for mode 6

	//We multiply the analog trigger reads by this to shut them off if the trigger is acting as another button
	const int shutoffLa = (controls.lRemap == (1 << L_REMAP)) ? 1 : 0;
	const int shutoffRa = (controls.rRemap == (1 << R_REMAP)) ? 1 : 0;

	//These are used for mode 7, but they're calculated out here so we can scale the deadzone too.
	float triggerScaleL = (0.0112f * controls.lTriggerOffset) + 0.4494f;
	float triggerScaleR = (0.0112f * controls.rTriggerOffset) + 0.4494f;

	switch(controls.lConfig) {
		case 0: //Default Trigger state
			tempBtn.La = (uint8_t) readLa(pin, controls.lTrigInitial, 1) * shutoffLa;
			break;
		case 1: //Digital Only Trigger state
			tempBtn.La = (uint8_t) 0;
			break;
		case 2: //Analog Only Trigger state
			tempBtn.L  = (uint8_t) 0;
			tempBtn.La = (uint8_t) readLa(pin, controls.lTrigInitial, 1) * shutoffLa;
			break;
		case 3: //Trigger Plug Emulation state
			if(!tempBtn.L) {
				tempBtn.La = (uint8_t) fmin(controls.lTriggerOffset, readLa(pin, controls.lTrigInitial, 1) * shutoffLa);
			} else {
				tempBtn.La = (uint8_t) 255;
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
			tempBtn.La = (uint8_t) readLa(pin, controls.lTrigInitial, triggerScaleL) * shutoffLa;
			break;
		default:
			tempBtn.La = (uint8_t) readLa(pin, controls.lTrigInitial, 1) * shutoffLa;
	}

	switch(controls.rConfig) {
		case 0: //Default Trigger state
			tempBtn.Ra = (uint8_t) readRa(pin, controls.rTrigInitial, 1) * shutoffRa;
			break;
		case 1: //Digital Only Trigger state
			tempBtn.Ra = (uint8_t) 0;
			break;
		case 2: //Analog Only Trigger state
			tempBtn.R  = (uint8_t) 0;
			tempBtn.Ra = (uint8_t) readRa(pin, controls.rTrigInitial, 1) * shutoffRa;
			break;
		case 3: //Trigger Plug Emulation state
			if(!tempBtn.R) {
				tempBtn.Ra = (uint8_t) fmin(controls.rTriggerOffset, readRa(pin, controls.rTrigInitial, 1) * shutoffRa);
			} else {
				tempBtn.Ra = (uint8_t) 255;
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
			tempBtn.Ra = (uint8_t) readRa(pin, controls.rTrigInitial, triggerScaleR) * shutoffRa;
			break;
		default:
			tempBtn.Ra = (uint8_t) readRa(pin, controls.rTrigInitial, 1) * shutoffRa;
	}

	//Apply any further button remapping to tempBtn here

	//Tournament toggle
	static int startLockout = 1000;
	if(controls.tournamentToggle >= 3 && hardware.S) {
		if(startLockout > 0) {
			startLockout--;
			tempBtn.S = (uint8_t) (0);
		} else if(startLockout <= 0) {
			tempBtn.S = (uint8_t) (1);
		}
		//override and instantly output start if both start and ddown are pressed
		if(hardware.Dd) {
			tempBtn.S = (uint8_t) (1);
			//suppress ddown
			tempBtn.Dd = (uint8_t) (0);
		}
	} else if(startLockout < 1000) {
		startLockout++;
	}
	static int duLockout = 1000;
	if((controls.tournamentToggle == 1 || controls.tournamentToggle == 4) && hardware.Du) {
		if(duLockout > 0) {
			duLockout--;
			tempBtn.Du = (uint8_t) (0);
		} else if(duLockout <= 0) {
			tempBtn.Du = (uint8_t) (1);
		}
		//override and instantly dup if start and dup are both pressed
		if(hardware.S) {
			tempBtn.Du = (uint8_t) (1);
			//suppress start
			tempBtn.S = (uint8_t) (0);
		}
	} else if(duLockout < 1000) {
		duLockout++;
	}
	if(controls.tournamentToggle == 2 || controls.tournamentToggle == 5) {
		//override the disable and dup if start and dup are both pressed
		if(hardware.S && hardware.Du) {
			tempBtn.Du = (uint8_t) (1);
			//suppress start
			tempBtn.S = (uint8_t) (0);
		} else {
			tempBtn.Du = (uint8_t) (0);
		}
	}

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
	* Tournament Toggle:  Z+Start+Du
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
	* Toggle raw stick output: LR+Du
	*
	* Analog Stick Configuration:
	* Increase/Decrease X-Axis Snapback Filtering:  AX+Du/Dd
	* Increase/Decrease Y-Axis Snapback Filtering:  AY+Du/Dd
	* Increase/Decrease X-Axis Waveshaping:  LX+Du/Dd
	* Increase/Decrease Y-Axis Waveshaping:  LY+Du/Dd
	* Increase/Decrease X-Axis Smoothing:  RX+Du/Dd
	* Increase/Decrease Y-Axis Smoothing:  RY+Du/Dd
	* Show Analog Filtering Settings: L+Start
	* Increase/Decrease Analog Scaler: LA+Du/Dd
	* Increase/Decrease Cardinal Snapping: RA+Du/Dd
	*
	* C-Stick Configuration
	* Increase/Decrease X-Axis Snapback Filtering:  AXZ+Du/Dd
	* Increase/Decrease Y-Axis Snapback Filtering:  AYZ+Du/Dd
	* Increase/Decrease X-Axis Waveshaping:  LXZ+Du/Dd
	* Increase/Decrease X-Axis Waveshaping:  LXZ+Du/Dd
	* Increase/Decrease X-Axis Offset:  RXZ+Du/Dd
	* Increase/Decrease Y-Axis Offset:  RYZ+Du/Dd
	* Show C-Stick Settings:  R+Start
	* Increase/Decrease Analog Scaler: LAZ+Du/Dd
	* Increase/Decrease Cardinal Snapping: RAZ+Du/Dd
	*
	* Remap: BXY without A, then press ABLRXYZ in order
	* Cancel all remaps: BXR, without A
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
	bool beginRemapping = false;

	//This will count up as we request settings changes continuously
	//If we enter the following if else block with a nonzero counter but no commands are used,
	// then that means we are done changing (for now) and can commit.
	//Primarily meant for the trigger offset setting, which has a lot of changes.
	static int settingChangeCount = 0;

	//check the hardware buttons to change the controller settings
	if(!controls.safeMode && (currentCalStep == -1) && (currentRemapStep == -1)) {
		//it'll be unlocked after it hits zero
		const int hardResetLockoutDuration = 800;
		static int hardResetLockout = hardResetLockoutDuration;
		if(hardware.A && hardware.B && hardware.Z && hardware.Dd) { //Hard Reset pressed
			if(hardResetLockout > 0) { //Not held long enough
				hardResetLockout--;
			} else if(hardResetLockout == 0) { //Held long enough
				hardResetLockout = hardResetLockoutDuration;
				resetDefaults(HARD, controls, gains, normGains, _aStickParams, _cStickParams);//do reset sticks
				freezeSticks(2000, btn, hardware);
			}
		} else if(hardResetLockout < hardResetLockoutDuration) {
			hardResetLockout++;
		}

		if(hardware.A && hardware.X && hardware.Y && hardware.S && !hardware.L && !hardware.R) { //Safe Mode Toggle
			controls.safeMode = true;
			currentlyRaw = false;
			freezeSticks(4000, btn, hardware);
		} else if (hardware.A && hardware.Z && hardware.Du && !hardware.X && !hardware.Y && !hardware.L && !hardware.R) { //display version number (ignore commands for c stick snapback)
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
			//actually do nothing, this is just to prevent other things from happening
		} else if (hardware.A && hardware.X && hardware.Y && hardware.Z) { //Toggle Auto-Initialize
			settingChangeCount++;
			changeAutoInit(btn, hardware, controls);
		} else if(hardware.Z && hardware.S && hardware.Du && !hardware.A && !hardware.B && !hardware.X && !hardware.Y) {
			settingChangeCount++;
			changeTournamentToggle(btn, hardware, controls);
		} else if (hardware.A && hardware.B && hardware.Du) { //Increase Rumble
			settingChangeCount++;
#ifdef RUMBLE
			changeRumble(INCREASE, btn, hardware, controls);
#else // RUMBLE
			//nothing
			freezeSticks(2000, btn, hardware);
#endif // RUMBLE
		} else if (hardware.A && hardware.B && hardware.Dd) { //Decrease Rumble
			settingChangeCount++;
#ifdef RUMBLE
			changeRumble(DECREASE, btn, hardware, controls);
#else // RUMBLE
			//nothing
			freezeSticks(2000, btn, hardware);
#endif // RUMBLE
		} else if (hardware.A && hardware.B && hardware.S) { //Show current rumble setting
			settingChangeCount++;
#ifdef RUMBLE
			showRumble(2000, btn, hardware, controls);
#else // RUMBLE
			freezeSticks(2000, btn, hardware);
#endif // RUMBLE
		} else if (hardware.A && hardware.X && hardware.Y && hardware.L) { //Analog Calibration
			debug_println("Calibrating the A stick");
			whichStick = ASTICK;
			currentCalStep = 0;
			advanceCal = true;
			freezeSticks(2000, btn, hardware);
		} else if (hardware.A && hardware.X && hardware.Y && hardware.R) { //C-stick Calibration
			debug_println("Calibrating the C stick");
			whichStick = CSTICK;
			currentCalStep = 0;
			advanceCal = true;
			freezeSticks(2000, btn, hardware);
		} else if(hardware.A && hardware.X && !hardware.Z && hardware.Du) { //Increase Analog X-Axis Snapback Filtering
			settingChangeCount++;
			adjustSnapback(XAXIS, INCREASE, btn, hardware, controls, gains, normGains);
		} else if(hardware.A && hardware.X && !hardware.Z && hardware.Dd) { //Decrease Analog X-Axis Snapback Filtering
			settingChangeCount++;
			adjustSnapback(XAXIS, DECREASE, btn, hardware, controls, gains, normGains);
		} else if(hardware.A && hardware.Y && !hardware.Z && hardware.Du) { //Increase Analog Y-Axis Snapback Filtering
			settingChangeCount++;
			adjustSnapback(YAXIS, INCREASE, btn, hardware, controls, gains, normGains);
		} else if(hardware.A && hardware.Y && !hardware.Z && hardware.Dd) { //Decrease Analog Y-Axis Snapback Filtering
			settingChangeCount++;
			adjustSnapback(YAXIS, DECREASE, btn, hardware, controls, gains, normGains);
		} else if(hardware.L && hardware.X && !hardware.Z && hardware.Du) { //Increase Analog X-Axis Waveshaping
			settingChangeCount++;
			adjustWaveshaping(ASTICK, XAXIS, INCREASE, btn, hardware, controls);
		} else if(hardware.L && hardware.X && !hardware.Z && hardware.Dd) { //Decrease Analog X-Axis Waveshaping
			settingChangeCount++;
			adjustWaveshaping(ASTICK, XAXIS, DECREASE, btn, hardware, controls);
		} else if(hardware.L && hardware.Y && !hardware.Z && hardware.Du) { //Increase Analog Y-Axis Waveshaping
			settingChangeCount++;
			adjustWaveshaping(ASTICK, YAXIS, INCREASE, btn, hardware, controls);
		} else if(hardware.L && hardware.Y && !hardware.Z && hardware.Dd) { //Decrease Analog Y-Axis Waveshaping
			settingChangeCount++;
			adjustWaveshaping(ASTICK, YAXIS, DECREASE, btn, hardware, controls);
		} else if(hardware.R && hardware.X && !hardware.Z && hardware.Du) { //Increase X-axis Delay
			settingChangeCount++;
			adjustSmoothing(XAXIS, INCREASE, btn, hardware, controls, gains, normGains);
		} else if(hardware.R && hardware.X && !hardware.Z && hardware.Dd) { //Decrease X-axis Delay
			settingChangeCount++;
			adjustSmoothing(XAXIS, DECREASE, btn, hardware, controls, gains, normGains);
		} else if(hardware.R && hardware.Y && !hardware.Z && hardware.Du) { //Increase Y-axis Delay
			settingChangeCount++;
			adjustSmoothing(YAXIS, INCREASE, btn, hardware, controls, gains, normGains);
		} else if(hardware.R && hardware.Y && !hardware.Z && hardware.Dd) { //Decrease Y-axis Delay
			settingChangeCount++;
			adjustSmoothing(YAXIS, DECREASE, btn, hardware, controls, gains, normGains);
		} else if(hardware.R && hardware.A && hardware.Du && !hardware.Z) { //Increase Cardinal Snapping
			settingChangeCount++;
			adjustCardinalSnapping(ASTICK, INCREASE, btn, hardware, controls);
		} else if(hardware.R && hardware.A && hardware.Dd && !hardware.Z) { //Decrease Cardinal Snapping
			settingChangeCount++;
			adjustCardinalSnapping(ASTICK, DECREASE, btn, hardware, controls);
		} else if(hardware.L && hardware.A && hardware.Du && !hardware.Z) { //Increase Analog Scaler
			settingChangeCount++;
			adjustAnalogScaler(ASTICK, INCREASE, btn, hardware, controls);
		} else if(hardware.L && hardware.A && hardware.Dd && !hardware.Z) { //Decrease Analog Scaler
			settingChangeCount++;
			adjustAnalogScaler(ASTICK, DECREASE, btn, hardware, controls);
		} else if(hardware.L && hardware.S && !hardware.A && !hardware.R && !hardware.X && !hardware.Y) { //Show Current Analog Settings (ignore L remap and L trigger toggle and LRAS)
			showAstickSettings(btn, hardware, controls, gains);
		} else if(hardware.A && hardware.X && hardware.Z && hardware.Du) { //Increase C-stick X-Axis Snapback Filtering
			settingChangeCount++;
			adjustCstickSmoothing(XAXIS, INCREASE, btn, hardware, controls, gains, normGains);
		} else if(hardware.A && hardware.X && hardware.Z && hardware.Dd) { //Decrease C-stick X-Axis Snapback Filtering
			settingChangeCount++;
			adjustCstickSmoothing(XAXIS, DECREASE, btn, hardware, controls, gains, normGains);
		} else if(hardware.A && hardware.Y && hardware.Z && hardware.Du) { //Increase C-stick Y-Axis Snapback Filtering
			settingChangeCount++;
			adjustCstickSmoothing(YAXIS, INCREASE, btn, hardware, controls, gains, normGains);
		} else if(hardware.A && hardware.Y && hardware.Z && hardware.Dd) { //Decrease C-stick Y-Axis Snapback Filtering
			settingChangeCount++;
			adjustCstickSmoothing(YAXIS, DECREASE, btn, hardware, controls, gains, normGains);
		} else if(hardware.L && hardware.X && hardware.Z && hardware.Du) { //Increase C-stick X-Axis Waveshaping
			settingChangeCount++;
			adjustWaveshaping(CSTICK, XAXIS, INCREASE, btn, hardware, controls);
		} else if(hardware.L && hardware.X && hardware.Z && hardware.Dd) { //Decrease C-stick X-Axis Waveshaping
			settingChangeCount++;
			adjustWaveshaping(CSTICK, XAXIS, DECREASE, btn, hardware, controls);
		} else if(hardware.L && hardware.Y && hardware.Z && hardware.Du) { //Increase C-stick Y-Axis Waveshaping
			settingChangeCount++;
			adjustWaveshaping(CSTICK, YAXIS, INCREASE, btn, hardware, controls);
		} else if(hardware.L && hardware.Y && hardware.Z && hardware.Dd) { //Decrease C-stick Y-Axis Waveshaping
			adjustWaveshaping(CSTICK, YAXIS, DECREASE, btn, hardware, controls);
		} else if(hardware.R && hardware.X && hardware.Z && hardware.Du) { //Increase C-stick X Offset
			settingChangeCount++;
			adjustCstickOffset(XAXIS, INCREASE, btn, hardware, controls);
		} else if(hardware.R && hardware.X && hardware.Z && hardware.Dd) { //Decrease C-stick X Offset
			settingChangeCount++;
			adjustCstickOffset(XAXIS, DECREASE, btn, hardware, controls);
		} else if(hardware.R && hardware.Y && hardware.Z && hardware.Du) { //Increase C-stick Y Offset
			settingChangeCount++;
			adjustCstickOffset(YAXIS, INCREASE, btn, hardware, controls);
		} else if(hardware.R && hardware.Y && hardware.Z && hardware.Dd) { //Decrease C-stick Y Offset
			settingChangeCount++;
			adjustCstickOffset(YAXIS, DECREASE, btn, hardware, controls);
		} else if(hardware.R && hardware.A && hardware.Z && hardware.Du) { //Increase C-stick Cardinal Snapping
			settingChangeCount++;
			adjustCardinalSnapping(CSTICK, INCREASE, btn, hardware, controls);
		} else if(hardware.R && hardware.A && hardware.Z && hardware.Dd) { //Decrease C-stick Cardinal Snapping
			settingChangeCount++;
			adjustCardinalSnapping(CSTICK, DECREASE, btn, hardware, controls);
		} else if(hardware.L && hardware.A && hardware.Z && hardware.Du) { //Increase C-stick Analog Scaler
			settingChangeCount++;
			adjustAnalogScaler(CSTICK, INCREASE, btn, hardware, controls);
		} else if(hardware.L && hardware.A && hardware.Z && hardware.Dd) { //Decrease C-stick Analog Scaler
			settingChangeCount++;
			adjustAnalogScaler(CSTICK, DECREASE, btn, hardware, controls);
		} else if(hardware.R && hardware.S && !hardware.A && !hardware.L && !hardware.X && !hardware.Y) { //Show Current C-stick Settings (ignore R remap and R trigger toggle and LRAS)
			showCstickSettings(btn, hardware, controls, gains);
		} else if(hardware.A && hardware.B && hardware.L) { //Toggle Analog L
			settingChangeCount++;
			nextTriggerState(LTRIGGER, btn, hardware, controls);
		} else if(hardware.A && hardware.B && hardware.R) { //Toggle Analog R
			settingChangeCount++;
			nextTriggerState(RTRIGGER, btn, hardware, controls);
		} else if(hardware.L && hardware.B && hardware.Du) { //Increase L-Trigger Offset
			settingChangeCount++;
			adjustTriggerOffset(LTRIGGER, INCREASE, btn, hardware, controls);
		} else if(hardware.L && hardware.B && hardware.Dd) { //Decrease L-trigger Offset
			settingChangeCount++;
			adjustTriggerOffset(LTRIGGER, DECREASE, btn, hardware, controls);
		} else if(hardware.R && hardware.B && hardware.Du) { //Increase R-trigger Offset
			settingChangeCount++;
			adjustTriggerOffset(RTRIGGER, INCREASE, btn, hardware, controls);
		} else if(hardware.R && hardware.B && hardware.Dd) { //Decrease R-trigger Offset
			settingChangeCount++;
			adjustTriggerOffset(RTRIGGER, DECREASE, btn, hardware, controls);
		} else if(hardware.B && hardware.X && hardware.Y && !hardware.A) { //Initiate remapping
			debug_println("Remapping buttons");
			beginRemapping = true;
			freezeSticks(2000, btn, hardware);
		} else if(hardware.B && hardware.R && hardware.X && !hardware.A) { //Reset remapping
			resetRemap(controls);
			freezeSticks(2000, btn, hardware);
		} else if(hardware.L && hardware.R && hardware.Du) {
			currentlyRaw = !currentlyRaw;
			freezeSticks(500, btn, hardware);
		} else if(checkAdjustExtra(EXTRAS_UP, btn, false)) { // Toggle Extras
			settingChangeCount++;
			toggleExtra(EXTRAS_UP, btn, hardware, controls);
		} else if(checkAdjustExtra(EXTRAS_DOWN, btn, false)) {
			settingChangeCount++;
			toggleExtra(EXTRAS_DOWN, btn, hardware, controls);
		} else if(checkAdjustExtra(EXTRAS_LEFT, btn, false)) {
			settingChangeCount++;
			toggleExtra(EXTRAS_LEFT, btn, hardware, controls);
		} else if(checkAdjustExtra(EXTRAS_RIGHT, btn, false)) {
			settingChangeCount++;
			toggleExtra(EXTRAS_RIGHT, btn, hardware, controls);
		} else if(checkAdjustExtra(EXTRAS_UP, btn, true)) { // Configure Extras
			settingChangeCount++;
			configExtra(EXTRAS_UP, btn, hardware, controls);
		} else if(checkAdjustExtra(EXTRAS_DOWN, btn, true)) {
			settingChangeCount++;
			configExtra(EXTRAS_DOWN, btn, hardware, controls);
		} else if(checkAdjustExtra(EXTRAS_LEFT, btn, true)) {
			settingChangeCount++;
			configExtra(EXTRAS_LEFT, btn, hardware, controls);
		} else if(checkAdjustExtra(EXTRAS_RIGHT, btn, true)) {
			settingChangeCount++;
			configExtra(EXTRAS_RIGHT, btn, hardware, controls);
		} else {
			//If the buttons were released after changing an applicable setting
			if(settingChangeCount > 0) {
				settingChangeCount = 0;
				//request a commit only if we need to batch them.
#ifdef BATCHSETTINGS
				commitSettings();
#endif //BATCHSETTINGS
			}
		}
	} else if ((currentCalStep == -1) && (currentRemapStep == -1)) { //Safe Mode Enabled, Lock Settings, wait for safe mode command

		//it'll be unlocked after it hits zero
		const int safeModeLockoutDuration = 800;
		static int safeModeLockout = safeModeLockoutDuration;
		if(hardware.A && hardware.X && hardware.Y && hardware.S && !hardware.L && !hardware.R) { //Safe Mode toggle
			if(safeModeLockout > 0) { //Not held long enough
				safeModeLockout--;
			} else if(safeModeLockout == 0) { //Held long enough
				safeModeLockout = safeModeLockoutDuration;
				if(!running) { //wake it up if not already running
					running = true;
				}
				controls.safeMode = false;
				freezeSticks(2000, btn, hardware);
			}
		} else if(safeModeLockout < safeModeLockoutDuration) {
			safeModeLockout++;
		}
	}

	//Skip stick measurement and go to notch adjust using the start button while calibrating
	if(hardware.S && (currentCalStep >= 0 && currentCalStep < 32)){
		calibrationSkipMeasurement(currentCalStep, whichStick, tempCalPointsX, tempCalPointsY, notchStatus, notchAngles, measuredNotchAngles, aStickParams, cStickParams, controls);
	}

	//Undo Calibration using Z-button
	static bool undoCal = false;
	static bool undoCalPressed = false;
	if(hardware.Z && undoCal && !undoCalPressed) {
		undoCalPressed = true;
		calibrationUndo(currentCalStep, whichStick, notchStatus);
	} else if(!hardware.Z) {
		undoCalPressed = false;
	}

	//Advance Calibration Using L or R triggers or A button
	static int calibLockout = 50;
	static bool advanceCalPressed = false;
	if(advanceCal) {
		if(hardware.A || hardware.L || hardware.R) {
			if(calibLockout >= 50) {
				calibLockout = 0;
			} else {
				calibLockout = max(0, calibLockout - 1);
			}
		} else {
			calibLockout = min(50, calibLockout + 1);
		}
		if(calibLockout < 50 && !advanceCalPressed) {
			advanceCalPressed = true;
			calibrationAdvance(controls, currentCalStep, whichStick, tempCalPointsX, tempCalPointsY, undoCal, notchAngles, notchStatus, measuredNotchAngles, aStickParams, cStickParams);
			if(currentCalStep == -1) {
				advanceCal = false;
			}
		} else if (calibLockout >= 50) {
			advanceCalPressed = false;
		}
	}

	if(beginRemapping) {
		remapAdvance(currentRemapStep, controls, hardware, btn);
		beginRemapping = false;
	}

	if((currentRemapStep != -1) && !controls.safeMode) {
		if(hardware.A || hardware.B || hardware.L || hardware.R ||
				hardware.X || hardware.Y || hardware.Z) {
			remapAdvance(currentRemapStep, controls, hardware, btn);
		}
	}
}

void readSticks(int readA, int readC, Buttons &btn, Pins &pin, RawStick &raw, const Buttons &hardware, const ControlConfig &controls, const FilterGains &normGains, const StickParams &aStickParams, const StickParams &cStickParams, float &dT, const int currentCalStep, const bool currentlyRaw){
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

//#ifndef CLEANADC
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

	//debug_println(adcCount);
	float aStickX = aXSum/(float)adcCount/4096.0*_ADCScale;
	float aStickY = aYSum/(float)adcCount/4096.0*_ADCScale;
	float cStickX = cXSum/(float)adcCount/4096.0*_ADCScale;
	float cStickY = cYSum/(float)adcCount/4096.0*_ADCScale;
	/*
#else //CLEANADC: read only once
	float aStickX = readAx(pin)/4096.0;
	float aStickY = readAy(pin)/4096.0;
	float cStickX = readCx(pin)/4096.0;
	float cStickY = readCy(pin)/4096.0;
	//note: this actually results in about 0.5 ms delay for the analog sticks

	uint32_t thisMicros = micros();
	while(thisMicros-lastMicros < 1000) {
		thisMicros = micros();
	}
#endif //CLEANADC
	*/
	dT = (micros()-lastMicros)/1000;
	lastMicros += 1000;
	if(micros() - lastMicros > 1500) { //handle the case that it was synced and now isn't
		lastMicros = micros();
	}
	raw.axRaw = aStickX;
	raw.ayRaw = aStickY;
	raw.cxRaw = cStickX;
	raw.cyRaw = cStickY;

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

	raw.axLinearized = posAx;
	raw.ayLinearized = posAy;
	raw.cxLinearized = posCx;
	raw.cyLinearized = posCy;

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
	notchRemap(posAx, posAy, &remappedAx, &remappedAy, _noOfNotches, aStickParams, currentCalStep, controls, ASTICK);
	notchRemap(posCx, posCy, &remappedCx, &remappedCy, _noOfNotches, cStickParams, currentCalStep, controls, CSTICK);
	notchRemap(raw.axLinearized, raw.ayLinearized, &remappedAxUnfiltered, &remappedAyUnfiltered, _noOfNotches, aStickParams, 1, controls, ASTICK);//no snapping
	notchRemap(raw.cxLinearized, raw.cyLinearized, &remappedCxUnfiltered, &remappedCyUnfiltered, _noOfNotches, cStickParams, 1, controls, CSTICK);//no snapping

	//Clamp values from -125 to +125
	remappedAx = fmin(125, fmax(-125, remappedAx));
	remappedAy = fmin(125, fmax(-125, remappedAy));
	remappedCx = fmin(125, fmax(-125, remappedCx));
	remappedCy = fmin(125, fmax(-125, remappedCy));
	raw.axUnfiltered = fmin(125, fmax(-125, remappedAxUnfiltered));
	raw.ayUnfiltered = fmin(125, fmax(-125, remappedAyUnfiltered));
	raw.cxUnfiltered = fmin(125, fmax(-125, remappedCxUnfiltered));
	raw.cyUnfiltered = fmin(125, fmax(-125, remappedCyUnfiltered));

	bool skipAHyst = false;
#ifdef EXTRAS_ESS
	//ESS adapter functionality for Ocarina of Time on WiiVC if enabled
	skipAHyst = ess::remap(&remappedAx, &remappedAy, controls.extras[ess::extrasEssConfigSlot].config);
#endif

	//output raw either if it's been requested, or if the stick in question is completely uncalibrated (from a hard reset or when just initialized)
	const bool aRaw = currentlyRaw || ((aStickParams.fitCoeffsX[0] == 0) && (aStickParams.fitCoeffsY[0] == 0));
	const bool cRaw = currentlyRaw || ((cStickParams.fitCoeffsX[0] == 0) && (cStickParams.fitCoeffsY[0] == 0));

	float hystVal = 0.3;
	//assign the remapped values to the button struct
	if(readA){
		if(!aRaw) {
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
		} else {
			btn.Ax = (uint8_t) (_floatOrigin + aStickX*100);
			btn.Ay = (uint8_t) (_floatOrigin + aStickY*100);
		}
	}
	if(readC){
		if(!cRaw) {
			float diffCx = (remappedCx+_floatOrigin)-btn.Cx;
			if( (diffCx > (1.0 + hystVal)) || (diffCx < -hystVal) ){
				btn.Cx = (uint8_t) (remappedCx+_floatOrigin);
			}
			float diffCy = (remappedCy+_floatOrigin)-btn.Cy;
			if( (diffCy > (1.0 + hystVal)) || (diffCy < -hystVal) ){
				btn.Cy = (uint8_t) (remappedCy+_floatOrigin);
			}
		} else {
			btn.Cx = (uint8_t) (_floatOrigin + cStickX*100);
			btn.Cy = (uint8_t) (_floatOrigin + cStickY*100);
		}
	}
};

#endif //PHOBGCC_H
