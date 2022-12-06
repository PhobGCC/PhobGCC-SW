#ifndef FILTER_H
#define FILTER_H

#include <cmath>

#include "structsAndEnums.h"

//The median filter can be either length 3, 4, or 5.
#define MEDIANLEN 5
//Edit MEDIANARRAY to be MEDIANLEN long
#define MEDIANARRAY {0,0,0,0,0}
//Comment this define to disable it entirely.
//#define USEMEDIAN

void runMedian(float &val, float valArray[MEDIANLEN], unsigned int &medianIndex){
	//takes the value, inserts it into the value array, and then
	// writes the median back to the value
	valArray[medianIndex] = val;
	medianIndex = (medianIndex + 1) % MEDIANLEN;

	//We'll hardcode different sort versions according to how long the median is
	//These are derived from RawTherapee's median.h.
#if MEDIANLEN == 3
	val = fmax(fmin(valArray[0], valArray[1]), fmin(valArray[2], fmax(valArray[0], valArray[1])));
#elif MEDIANLEN == 4
	float maximin = fmax(fmin(valArray[0], valArray[1]), fmin(valArray[2], valArray[3]));
	float minimax = fmin(fmax(valArray[0], valArray[1]), fmax(valArray[2], valArray[3]));
	val = (maximin + minimax) / 2.0f;
#else //MEDIANLEN == 5
	float tmpArray[MEDIANLEN];
	float tmp;
	tmp         = fmin(valArray[0], valArray[1]);
	tmpArray[1] = fmax(valArray[0], valArray[1]);
	tmpArray[0] = tmp;
	tmp         = fmin(valArray[3], valArray[4]);
	tmpArray[4] = fmax(valArray[3], valArray[4]);
	tmpArray[3] = fmax(tmpArray[0], tmp);
	tmpArray[1] = fmin(tmpArray[1], tmpArray[4]);
	tmp         = fmin(tmpArray[1], valArray[2]);
	tmpArray[2] = fmax(tmpArray[1], valArray[2]);
	tmpArray[1] = tmp;
	tmp         = fmin(tmpArray[2], tmpArray[3]);
	val         = fmax(tmpArray[1], tmp);
#endif
}

void recomputeGains(const FilterGains &gains, FilterGains &normGains){
	//Recompute the intermediate gains used directly by the kalman filter
	//This happens according to the time between loop iterations.
	//Before, this happened every iteration of runKalman, but now
	//the event loop runs at a fixed 1000 Hz
	//Even if it's not *exactly* 1000 Hz, it should be constant enough.
	//Hopefully.
	//So now, this should be called any time gains gets changed.
	const float timeFactor = 1.0 / 1.2;
	const float timeDivisor = 1.2 / 1.0;
	normGains.maxStick      = gains.maxStick*gains.maxStick;//we actually use the square
	normGains.xVelDecay     = gains.xVelDecay      * timeFactor;
	normGains.yVelDecay     = gains.yVelDecay      * timeFactor;
	normGains.xVelPosFactor = gains.xVelPosFactor  * timeFactor;
	normGains.yVelPosFactor = gains.yVelPosFactor  * timeFactor;
	normGains.xVelDamp      = gains.xVelDamp       * timeDivisor;
	normGains.yVelDamp      = gains.yVelDamp       * timeDivisor;
	normGains.velThresh     = 1/(gains.velThresh   * timeFactor);//slight optimization by using the inverse
	normGains.accelThresh   = 1/(gains.accelThresh * timeFactor);
	normGains.velThresh     = normGains.velThresh*normGains.velThresh;//square it because it's used squared
	normGains.accelThresh   = normGains.accelThresh*normGains.accelThresh;
	normGains.xSmoothing    = pow(1-gains.xSmoothing, timeDivisor);
	normGains.ySmoothing    = pow(1-gains.ySmoothing, timeDivisor);
	normGains.cXSmoothing   = pow(1-gains.cXSmoothing, timeDivisor);
	normGains.cYSmoothing   = pow(1-gains.cYSmoothing, timeDivisor);
};

void runKalman(float &xPosFilt, float &yPosFilt, const float xZ,const float yZ, const ControlConfig &controls, const FilterGains &normGains){
	//Serial.println("Running Kalman");

	//kalman filter state variables saved across iterations
	static float xPos = 0;//input of kalman filter
	static float yPos = 0;//input of kalman filter
	static float xVel = 0;
	static float yVel = 0;
	static float xVelFilt = 0;
	static float yVelFilt = 0;

	//save previous values of state
	const float oldXPos = xPos;
	const float oldYPos = yPos;
	const float oldXPosFilt = xPosFilt;
	const float oldYPosFilt = yPosFilt;
	const float oldXVel = xVel;
	const float oldYVel = yVel;
	const float oldXVelFilt = xVelFilt;
	const float oldYVelFilt = yVelFilt;

	//compute new (more trivial) state
	xPos = xZ;
	yPos = yZ;
	xVel = xPos - oldXPos;
	yVel = yPos - oldYPos;
	const float xVelSmooth = 0.5*(xVel + oldXVel);
	const float yVelSmooth = 0.5*(yVel + oldYVel);
	const float xAccel = xVel - oldXVel;
	const float yAccel = yVel - oldYVel;
	const float oldXPosDiff = oldXPos - oldXPosFilt;
	const float oldYPosDiff = oldYPos - oldYPosFilt;

	//compute stick position exponents for weights
	const float stickDistance2 = fmin(normGains.maxStick, xPos*xPos + yPos*yPos)/normGains.maxStick;//0-1
	const float stickDistance6 = stickDistance2*stickDistance2*stickDistance2;

	//the current velocity weight for the filtered velocity is the stick r^2
	const float velWeight1 = stickDistance2;
	const float velWeight2 = 1-velWeight1;

	//modified velocity to feed into our kalman filter.
	//We don't actually want an accurate model of the velocity, we want to suppress snapback without adding delay
	//term 1: weight current velocity according to r^2
	//term 2: the previous filtered velocity, weighted the opposite and also set to decay
	//term 3: a corrective factor based on the disagreement between real and filtered position

	//the current position weight used for the filtered position is whatever is larger of
	//  a) 1 minus the sum of the squares of
	//    1) the smoothed velocity divided by the velocity threshold
	//    2) the acceleration divided by the accel threshold
	//  b) stick r^6
	//When the stick is moving slowly, we want to weight it highly, in order to achieve
	//  quick control for inputs such as tilts. We lock out using both velocity and
	//  acceleration in order to rule out snapback.
	//When the stick is near the rim, we also want instant response, and we know snapback
	//  doesn't reach the rim.

	//In calculating the filtered stick position, we have the following components
	//term 1: current position, weighted according to the above weight
	//term 2: a predicted position based on the filtered velocity and previous filtered position,
	//  with the filtered velocity damped, and the overall term weighted inverse of the previous term
	//term 3: the integral error correction term

	//But if we xSnapback or ySnapback is zero, we skip the calculation
	if(controls.xSnapback != 0){
		xVelFilt = velWeight1*xVel + (1-normGains.xVelDecay)*velWeight2*oldXVelFilt + normGains.xVelPosFactor*oldXPosDiff;

		const float xPosWeightVelAcc = 1 - fmin(1, xVelSmooth*xVelSmooth*normGains.velThresh + xAccel*xAccel*normGains.accelThresh);
		const float xPosWeight1 = fmax(xPosWeightVelAcc, stickDistance6);
		const float xPosWeight2 = 1-xPosWeight1;

		xPosFilt = xPosWeight1*xPos +
				   xPosWeight2*(oldXPosFilt + (1-normGains.xVelDamp)*xVelFilt);
	} else {
		xPosFilt = xPos;
	}

	if(controls.ySnapback != 0){
		yVelFilt = velWeight1*yVel + (1-normGains.yVelDecay)*velWeight2*oldYVelFilt + normGains.yVelPosFactor*oldYPosDiff;

		const float yPosWeightVelAcc = 1 - fmin(1, yVelSmooth*yVelSmooth*normGains.velThresh + yAccel*yAccel*normGains.accelThresh);
		const float yPosWeight1 = fmax(yPosWeightVelAcc, stickDistance6);
		const float yPosWeight2 = 1-yPosWeight1;

		yPosFilt = yPosWeight1*yPos +
				   yPosWeight2*(oldYPosFilt + (1-normGains.yVelDamp)*yVelFilt);
	} else {
		yPosFilt = yPos;
	}
};

//The input setting should range from 0 to 15.
//The output should be 0 for 0.
float calcWaveshapeMult(const int setting){
	if (setting > 0 && setting <= 5) {
		return 1.0/(440 - 40*setting);
	} else if (setting > 5 && setting <= 15) {
		return 1.0/(340 - 20*setting);
	} else {
		return 0;
	}
}

//This simulates an idealized sort of pode:
// if the stick is moving fast, it responds poorly, while
// if the stick is moving slowly, it follows closely.
//It's not suitable to be the sole filter, but when put after
// the smart snapback filter, it should be able to hold the
// output at the rim longer when released.
void aRunWaveShaping(const float xPos, const float yPos, float &xOut, float &yOut, const ControlConfig &controls, const FilterGains &normGains){
	volatile static float oldXPos = 0;
	volatile static float oldYPos = 0;
	volatile static float oldXVel = 0;
	volatile static float oldYVel = 0;

	volatile static float oldXOut = 0;
	volatile static float oldYOut = 0;

	const float xVel = xPos - oldXPos;
	const float yVel = yPos - oldYPos;
	const float xVelSmooth = 0.5*(xVel + oldXVel);
	const float yVelSmooth = 0.5*(yVel + oldYVel);

	//The lower this value, the stronger the effect.
	//Per Rienne's experimentation:
	//Minimum setting should be 500; this does nearly nothing
	//Sweetspot for higher stick speeds is 200-240
	//Sweetspot for lower stick speeds is 160-200
	//Frame timing goes weird around 160
	//max functional setting probably 80
	//extreme pode is like 32-80
	//32 should be the limit

	const float xFactor = calcWaveshapeMult(controls.axWaveshaping);
	const float yFactor = calcWaveshapeMult(controls.ayWaveshaping);

	const float oldXPosWeight = fmin(1, xVelSmooth*xVelSmooth*normGains.velThresh*xFactor);
	const float newXPosWeight = 1 - oldXPosWeight;
	const float oldYPosWeight = fmin(1, yVelSmooth*yVelSmooth*normGains.velThresh*yFactor);
	const float newYPosWeight = 1 - oldYPosWeight;

	xOut = oldXOut*oldXPosWeight + xPos*newXPosWeight;
	yOut = oldYOut*oldYPosWeight + yPos*newYPosWeight;

	oldXPos = xPos;
	oldYPos = yPos;
	oldXVel = xVel;
	oldYVel = yVel;

	oldXOut = xOut;
	oldYOut = yOut;
}
//We need to duplicate this and call each one only once so that we don't reuse the history and screw things up.
void cRunWaveShaping(const float xPos, const float yPos, float &xOut, float &yOut, const ControlConfig &controls, const FilterGains &normGains){
	volatile static float oldXPos = 0;
	volatile static float oldYPos = 0;
	volatile static float oldXVel = 0;
	volatile static float oldYVel = 0;

	volatile static float oldXOut = 0;
	volatile static float oldYOut = 0;

	const float xVel = xPos - oldXPos;
	const float yVel = yPos - oldYPos;
	const float xVelSmooth = 0.5*(xVel + oldXVel);
	const float yVelSmooth = 0.5*(yVel + oldYVel);

	//The lower this value, the stronger the effect.
	//Per Rienne's experimentation:
	//Minimum setting should be 500; this does nearly nothing
	//Sweetspot for higher stick speeds is 200-240
	//Sweetspot for lower stick speeds is 160-200
	//Frame timing goes weird around 160
	//max functional setting probably 80
	//extreme pode is like 32-80
	//32 should be the limit

	const float xFactor = calcWaveshapeMult(controls.cxWaveshaping);
	const float yFactor = calcWaveshapeMult(controls.cyWaveshaping);

	const float oldXPosWeight = fmin(1, xVelSmooth*xVelSmooth*normGains.velThresh*xFactor);
	const float newXPosWeight = 1 - oldXPosWeight;
	const float oldYPosWeight = fmin(1, yVelSmooth*yVelSmooth*normGains.velThresh*yFactor);
	const float newYPosWeight = 1 - oldYPosWeight;

	xOut = oldXOut*oldXPosWeight + xPos*newXPosWeight;
	yOut = oldYOut*oldYPosWeight + yPos*newYPosWeight;

	oldXPos = xPos;
	oldYPos = yPos;
	oldXVel = xVel;
	oldYVel = yVel;

	oldXOut = xOut;
	oldYOut = yOut;
}

#endif //FILTER_H
