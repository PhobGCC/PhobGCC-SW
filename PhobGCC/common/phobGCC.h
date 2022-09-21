#ifndef PHOBGCC_H
#define PHOBGCC_H

//Must be included at the end of the board header file, but before comms.h

#include "structsAndEnums.h"
#include "filter.h"
#include "stick.h"

//#define BUILD_RELEASE
#define BUILD_DEV

//This is just an integer.
#define SW_VERSION 26

//#define ENABLE_LED

Buttons _btn;

HardwareButtons _hardware;

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
int _rumblePower = calcRumblePower(_controls.rumble);

float velDampFromSnapback(const int snapback){
	return 0.125 * pow(2, (snapback-4)/3.0);//4 should yield 0.125, 10 should yield 0.5, don't care about 0
}

#endif //PHOBGCC_H
