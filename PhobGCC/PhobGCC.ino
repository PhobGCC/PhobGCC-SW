//This software uses bits of code from GoodDoge's Dogebawx project, which was the initial starting point: https://github.com/DogeSSBM/DogeBawx

#include <math.h>

#include "common/phobGCC.h"

extern "C" uint32_t set_arm_clock(uint32_t frequency);

void setup() {
    serialSetup();
	Serial.print("Software version 0.");
	Serial.println(SW_VERSION);
#ifdef BUILD_DEV
	Serial.println("This is not a stable version");
#endif

#ifdef TEENSY4_0
	//Force-underclock Teensy 4 to 150 MHz to lower power draw.
	//set_arm_clock(150'000'000);
	//for some reason, 150 MHz doesn't work with the new comms code. 300 seems to be enough.
	set_arm_clock(300'000'000);
#endif //TEENSY4_0

	const int numberOfNaN = readEEPROM(_controls, _gains, _normGains, _aStickParams, _cStickParams);
	Serial.print("Number of NaN in EEPROM: ");
	Serial.println(numberOfNaN);
	if(numberOfNaN > 10){//by default it seems 16 end up uninitialized on Teensy 4
		resetDefaults(FACTORY, _controls, _gains, _normGains, _aStickParams, _cStickParams);//do reset sticks
		readEEPROM(_controls, _gains, _normGains, _aStickParams, _cStickParams);
	}

	//set some of the unused values in the message response
	_btn.errS = 0;
	_btn.errL = 0;
	_btn.orig = 0;
	_btn.high = 1;

    setPinModes();

    ADCSetup(adc, _ADCScale, _ADCScaleFactor);

	//measure the trigger values for trigger tricking
	initializeButtons(_pinList, _btn, _controls.lTrigInitial, _controls.rTrigInitial);
	//set the origin response before the sticks have been touched
	//it will never be changed again after this

	//start up user-enabled extras
	extrasInit();

	commsSetup(_btn);
}

void loop() {
	static bool running = false;

	//check if we should be reporting values yet
	if((_btn.B || _controls.autoInit) && !running){
		Serial.println("Starting to report values");
		running=true;
	}

	static int currentCalStep = -1;//-1 means not calibrating

	//Set up persistent storage for calibration
	static float tempCalPointsX[_noOfCalibrationPoints];
	static float tempCalPointsY[_noOfCalibrationPoints];
	static WhichStick whichStick = ASTICK;
	static NotchStatus notchStatus[_noOfNotches];
	static float notchAngles[_noOfNotches];
	static float measuredNotchAngles[_noOfNotches];

	//read the controllers buttons
	processButtons(_pinList, _btn, _hardware, _controls, _gains, _normGains, currentCalStep, running, tempCalPointsX, tempCalPointsY, whichStick, notchStatus, notchAngles, measuredNotchAngles, _aStickParams, _cStickParams);

	//check to see if we are calibrating
	if(currentCalStep >= 0){
		if(whichStick == ASTICK){
			if(currentCalStep >= _noOfCalibrationPoints){//adjust notch angles
				adjustNotch(currentCalStep, _dT, ASTICK, measuredNotchAngles, notchAngles, notchStatus, _btn, _hardware);
				if(_hardware.Y || _hardware.X || _hardware.B){//only run this if the notch was adjusted
					//clean full cal points again, feeding updated angles in
					float cleanedPointsX[_noOfNotches+1];
					float cleanedPointsY[_noOfNotches+1];
					float notchPointsX[_noOfNotches+1];
					float notchPointsY[_noOfNotches+1];
					cleanCalPoints(tempCalPointsX, tempCalPointsY, notchAngles, cleanedPointsX, cleanedPointsY, notchPointsX, notchPointsY, notchStatus);
					//linearize again
					linearizeCal(cleanedPointsX, cleanedPointsY, cleanedPointsX, cleanedPointsY, _aStickParams);
					//notchCalibrate again to update the affine transform
					notchCalibrate(cleanedPointsX, cleanedPointsY, notchPointsX, notchPointsY, _noOfNotches, _aStickParams);
				}
			}else{//just show desired stick position
				displayNotch(currentCalStep, true, _notchAngleDefaults, _btn);
			}
			readSticks(true,false, _btn, _pinList, _raw, _hardware, _controls, _normGains, _aStickParams, _cStickParams, _dT, currentCalStep, false);
		}
		else{//WHICHSTICK == CSTICK
			if(currentCalStep >= _noOfCalibrationPoints){//adjust notch angles
				adjustNotch(currentCalStep, _dT, CSTICK, measuredNotchAngles, notchAngles, notchStatus, _btn, _hardware);
				if(_hardware.Y || _hardware.X || _hardware.B){//only run this if the notch was adjusted
					//clean full cal points again, feeding updated angles in
					float cleanedPointsX[_noOfNotches+1];
					float cleanedPointsY[_noOfNotches+1];
					float notchPointsX[_noOfNotches+1];
					float notchPointsY[_noOfNotches+1];
					cleanCalPoints(tempCalPointsX, tempCalPointsY, notchAngles, cleanedPointsX, cleanedPointsY, notchPointsX, notchPointsY, notchStatus);
					//linearize again
					linearizeCal(cleanedPointsX, cleanedPointsY, cleanedPointsX, cleanedPointsY, _cStickParams);
					//notchCalibrate again to update the affine transform
					notchCalibrate(cleanedPointsX, cleanedPointsY, notchPointsX, notchPointsY, _noOfNotches, _cStickParams);
				}
			}else{//just show desired stick position
				displayNotch(currentCalStep, false, _notchAngleDefaults, _btn);
			}
			readSticks(false,true, _btn, _pinList, _raw, _hardware, _controls, _normGains, _aStickParams, _cStickParams, _dT, currentCalStep, false);
		}
	}
	else if(running){
		//if not calibrating read the sticks normally
		readSticks(true,true, _btn, _pinList, _raw, _hardware, _controls, _normGains, _aStickParams, _cStickParams, _dT, currentCalStep, false);
	}
}
