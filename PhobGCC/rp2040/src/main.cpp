#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/timer.h"
#include "hardware/pwm.h"

#include "phobGCC.h"
#include "comms/joybus.hpp"
#include "cvideo.h"
#include "cvideo_variables.h"

volatile bool _vsyncSensors = false;
volatile bool _sync = false;

//This gets called by the comms library
GCReport __time_critical_func(buttonsToGCReport)() {
	GCReport report = {
		.a       = _btn.A,
		.b       = _btn.B,
		.x       = _btn.X,
		.y       = _btn.Y,
		.start   = _btn.S,
		.pad0    = 0,
		.dLeft   = _btn.Dl,
		.dRight  = _btn.Dr,
		.dDown   = _btn.Dd,
		.dUp     = _btn.Du,
		.z       = _btn.Z,
		.r       = _btn.L,
		.l       = _btn.R,
		.pad1    = 1,
		.xStick  = _btn.Ax,
		.yStick  = _btn.Ay,
		.cxStick = _btn.Cx,
		.cyStick = _btn.Cy,
		.analogL = _btn.La,
		.analogR = _btn.Ra
	};
	return report;
}

void second_core() {
	gpio_set_function(_pinLED, GPIO_FUNC_PWM);

	uint slice_num = pwm_gpio_to_slice_num(_pinLED);

	pwm_set_wrap(slice_num, 255);
	pwm_set_chan_level(slice_num, PWM_CHAN_B, 25);
	pwm_set_enabled(slice_num, true);

	extrasInit();

	//gpio_put(_pinSpare0, 0);

	while(true) { //main event loop

		//check if A is pressed to make it go at full speed
		/*
		if(_btn.A) {
			_vsyncSensors = false;
		} else {
			_vsyncSensors = true;
		}
		*/

		//limit speed if video is running
		if(_vsyncSensors) {
			while(!_sync) {
				tight_loop_contents();
			}
			_sync = false;
		}

		static bool running = false;

		gpio_put(_pinSpare0, !gpio_get_out_level(_pinSpare0));
		pwm_set_gpio_level(_pinLED, 255*gpio_get_out_level(_pinSpare0));

		//check if we should be reporting values yet
		if((_btn.B || _controls.autoInit) && !running){
			running=true;
		}

		//pwm_set_gpio_level(_pinLED, 255*_btn.B);

		static int currentCalStep = -1;//-1 means not calibrating

		//Set up persistent storage for calibration
		static float tempCalPointsX[_noOfCalibrationPoints];
		static float tempCalPointsY[_noOfCalibrationPoints];
		static WhichStick whichStick = ASTICK;
		static NotchStatus notchStatus[_noOfNotches];
		static float notchAngles[_noOfNotches];
		static float measuredNotchAngles[_noOfNotches];

		//check to see if we are calibrating
		if(currentCalStep >= 0){
			if(whichStick == ASTICK){
				if(currentCalStep >= _noOfCalibrationPoints){//adjust notch angles
					adjustNotch(currentCalStep, _dT, true, measuredNotchAngles, notchAngles, notchStatus, _btn, _hardware);
					if(_hardware.Y || _hardware.X || (_btn.B)){//only run this if the notch was adjusted
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
				readSticks(true,false, _btn, _pinList, _raw, _hardware, _controls, _normGains, _aStickParams, _cStickParams, _dT, currentCalStep, _vsyncSensors);
			}
			else{//WHICHSTICK == CSTICK
				if(currentCalStep >= _noOfCalibrationPoints){//adjust notch angles
					adjustNotch(currentCalStep, _dT, false, measuredNotchAngles, notchAngles, notchStatus, _btn, _hardware);
					if(_hardware.Y || _hardware.X || (_btn.B)){//only run this if the notch was adjusted
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
				readSticks(false,true, _btn, _pinList, _raw, _hardware, _controls, _normGains, _aStickParams, _cStickParams, _dT, currentCalStep, _vsyncSensors);
			}
		}
		else if(running){
			//if not calibrating read the sticks normally
			readSticks(true,true, _btn, _pinList, _raw, _hardware, _controls, _normGains, _aStickParams, _cStickParams, _dT, currentCalStep, _vsyncSensors);
		}

		//read the controller's buttons
		processButtons(_pinList, _btn, _hardware, _controls, _gains, _normGains, currentCalStep, running, tempCalPointsX, tempCalPointsY, whichStick, notchStatus, notchAngles, measuredNotchAngles, _aStickParams, _cStickParams);

	}
}

int main() {
	//set the clock speed to 125 kHz
	//the comms library needs this clockspeed
	//set_sys_clock_khz(1000*_us, true);

	//set up the main core so it can be paused by the other core
	//this is necessary for flash writing
	multicore_lockout_victim_init();

	//Read settings
	const int numberOfNaN = readEEPROM(_controls, _gains, _normGains, _aStickParams, _cStickParams);

	if(numberOfNaN > 10){//by default it seems 16 end up unitialized on pico
		resetDefaults(FACTORY, _controls, _gains, _normGains, _aStickParams, _cStickParams);
		readEEPROM(_controls, _gains, _normGains, _aStickParams, _cStickParams);
	}

	setPinModes();

	Pins pinList {//not actually necessary for any of the rp2040 read functions
		.pinLa = 0,
		.pinRa = 0,
		.pinL  = 0,
		.pinR  = 0,
		.pinAx = 0,
		.pinAy = 0,
		.pinCx = 0,
		.pinCy = 0,
		.pinRX = 0,
		.pinTX = 0,
		.pinDr = 0,
		.pinDu = 0,
		.pinDl = 0,
		.pinDd = 0,
		.pinX  = 0,
		.pinY  = 0,
		.pinA  = 0,
		.pinB  = 0,
		.pinZ  = 0,
		.pinS  = 0
	};

	_btn.A      =0;
	_btn.B      =0;
	_btn.X      =0;
	_btn.Y      =0;
	_btn.S      =0;
	_btn.orig   =0;
	_btn.errL   =0;
	_btn.errS   =0;
	_btn.Dl     =0;
	_btn.Dr     =0;
	_btn.Dd     =0;
	_btn.Du     =0;
	_btn.Z      =0;
	_btn.R      =0;
	_btn.L      =0;
	_btn.high   =1;
	_btn.Ax     =127;
	_btn.Ay     =127;
	_btn.Cx     =127;
	_btn.Cy     =127;
	_btn.La     =0;
	_btn.Ra     =0;
	_btn.magic1 =0;
	_btn.magic2 =0;

	_raw.axRaw = 0;
	_raw.ayRaw = 0;
	_raw.cxRaw = 0;
	_raw.cyRaw = 0;
	_raw.axUnfiltered = 0;
	_raw.ayUnfiltered = 0;
	_raw.cxUnfiltered = 0;
	_raw.cyUnfiltered = 0;

	readButtons(_pinList, _hardware);
	multicore_launch_core1(second_core);

	//Run comms unless Z is held while plugging in
	if(_hardware.Z) {
		_vsyncSensors = true;
		videoOut(_pinDac0, _btn, _hardware, _raw, _controls, _aStickParams, _cStickParams, _sync);
	} else {
		enterMode(_pinTX,
				_pinRumble,
				_pinBrake,
				_rumblePower,
				buttonsToGCReport);
	}

}
