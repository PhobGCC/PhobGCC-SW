#include "pico/stdlib.h"
#include "pico/bootrom.h"
#include "pico/multicore.h"
#include "hardware/timer.h"
#include "hardware/pwm.h"

#include "phobGCC.h"
#include "comms/joybus.hpp"
#include "cvideo.h"
#include "cvideo_variables.h"

volatile bool _videoOut = false;
//Variables used by PhobVision to communicate with the event loop core
volatile bool _sync = false;
volatile uint8_t _pleaseCommit = 0;//255 = redraw please
int _currentCalStep = -1;//-1 means not calibrating
DataCapture _dataCapture;

//This gets called by the comms library
GCReport __no_inline_not_in_flash_func(buttonsToGCReport)() {
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
		.r       = _btn.R,
		.l       = _btn.L,
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


	while(true) { //main event loop
		//Set up persistent storage for calibration
		static float tempCalPointsX[_noOfCalibrationPoints];
		static float tempCalPointsY[_noOfCalibrationPoints];
		static WhichStick whichStick = ASTICK;
		static NotchStatus notchStatus[_noOfNotches];
		static float notchAngles[_noOfNotches];
		static float measuredNotchAngles[_noOfNotches];
		static bool advanceCal = false;
		static bool undoCal = false;


		//when requested by the other core, commit the settings
		if(_pleaseCommit != 0) {
			if(_pleaseCommit == 1) {
				_pleaseCommit = 0;
				commitSettings();
			} else if(_pleaseCommit == 2) {
				_pleaseCommit = 0;
				resetDefaults(SOFT, _controls, _gains, _normGains, _aStickParams, _cStickParams);
			} else if(_pleaseCommit == 3) {
				_pleaseCommit = 0;
				resetDefaults(HARD, _controls, _gains, _normGains, _aStickParams, _cStickParams);
			} else if(_pleaseCommit == 4) {
				//Advance cal, A-stick
				_pleaseCommit = 255;
				whichStick = ASTICK;
				if(_currentCalStep == -1) {
					_currentCalStep++;
				}
				calibrationAdvance(_controls, _currentCalStep, whichStick, tempCalPointsX, tempCalPointsY, undoCal, notchAngles, notchStatus, measuredNotchAngles, _aStickParams, _cStickParams);
			} else if(_pleaseCommit == 5) {
				//Advance cal, C-stick
				_pleaseCommit = 255;
				whichStick = CSTICK;
				if(_currentCalStep == -1) {
					_currentCalStep++;
				}
				calibrationAdvance(_controls, _currentCalStep, whichStick, tempCalPointsX, tempCalPointsY, undoCal, notchAngles, notchStatus, measuredNotchAngles, _aStickParams, _cStickParams);
			} else if(_pleaseCommit == 6) {
				//undo cal
				_pleaseCommit = 255;
				calibrationUndo(_currentCalStep, whichStick, notchStatus);
			} else if(_pleaseCommit == 7) {
				//skip cal (might not actually get used?) (it's available but not shown)
				_pleaseCommit = 255;
				calibrationSkipMeasurement(_currentCalStep, whichStick, tempCalPointsX, tempCalPointsY, notchStatus, notchAngles, measuredNotchAngles, _aStickParams, _cStickParams, _controls);
			} else if(_pleaseCommit == 8) {
				_pleaseCommit = 0;
				_rumblePower = calcRumblePower(_controls.rumble);
				//rumble for 0.3 seconds
				pwm_set_gpio_level(_pinBrake, 0);
				pwm_set_gpio_level(_pinRumble, _rumblePower);
				int startTime = millis();
				int delta = 0;
				while(delta < 300) {
					delta = millis() - startTime;
				}
				pwm_set_gpio_level(_pinRumble, 0);
				pwm_set_gpio_level(_pinBrake, 255);
			} else if(_pleaseCommit == 9) {
				switch(_dataCapture.mode) {
					case CM_REACTION:
						//this expects that you initialize data capture by setting delay to 0
						_dataCapture.delay++;
						if(
								//_btn.arr[0] & 0b11110000 ||//abxy
								//_btn.arr[1] & 0b11111110 ||//dpad, lrz
								_btn.arr[0] & 0b00001111 ||//abxy
								_btn.arr[1] & 0b01111111 ||//dpad, lrz
								max(max(abs(int(_btn.Ax)-127), abs(int(_btn.Ay)-127)),
									max(abs(int(_btn.Cx)-127), abs(int(_btn.Cy)-127))) >= _dataCapture.stickThresh ||
								max(_btn.La, _btn.Ra) >= _dataCapture.triggerThresh) {
							_dataCapture.done = true;
							_pleaseCommit = 255;//end capture and display
						}
						break;
					case CM_STICK_RISING:
						//this begins capturing data immediately in a rolling buffer,
						//and on detecting a rising edge it sets the start point 20 ms in the past
						//this records the starting point only the first time, then it sets begin
						static int x0;
						static int y0;
						if(!_dataCapture.begin && !_dataCapture.done) {
							_dataCapture.begin = true;
							_dataCapture.done = false;
							_dataCapture.startIndex = 0;
							_dataCapture.endIndex = 255;//when it triggers, we freeze startIndex and start counting up here
							if(_dataCapture.captureStick == ASTICK) {
								x0 = _btn.Ax;
								y0 = _btn.Ay;
								_dataCapture.a1[_dataCapture.startIndex] = _btn.Ax;
								_dataCapture.a2[_dataCapture.startIndex] = _btn.Ay;
								_dataCapture.a1Unfilt[_dataCapture.startIndex] = (uint8_t) (_raw.axUnfiltered+_floatOrigin);
								_dataCapture.a2Unfilt[_dataCapture.startIndex] = (uint8_t) (_raw.ayUnfiltered+_floatOrigin);
							} else {
								x0 = _btn.Cx;
								y0 = _btn.Cy;
								_dataCapture.a1[_dataCapture.startIndex] = _btn.Cx;
								_dataCapture.a2[_dataCapture.startIndex] = _btn.Cy;
								_dataCapture.a1Unfilt[_dataCapture.startIndex] = (uint8_t) (_raw.cxUnfiltered+_floatOrigin);
								_dataCapture.a2Unfilt[_dataCapture.startIndex] = (uint8_t) (_raw.cyUnfiltered+_floatOrigin);
							}
							//                                                                   syxba                       lrz
							_dataCapture.abxyszrl[_dataCapture.startIndex] = (_btn.arr[0] & 0b00011111) | ((_btn.arr[1] & 0b01110000) << 1);
						} else {
							if(_dataCapture.endIndex == 255) {
								//not triggered yet
								//increment startIndex
								_dataCapture.startIndex = (_dataCapture.startIndex+1) % 100;
								//record
								if(_dataCapture.captureStick == ASTICK) {
									_dataCapture.a1[_dataCapture.startIndex] = _btn.Ax;
									_dataCapture.a2[_dataCapture.startIndex] = _btn.Ay;
									_dataCapture.a1Unfilt[_dataCapture.startIndex] = (uint8_t) (_raw.axUnfiltered+_floatOrigin);
									_dataCapture.a2Unfilt[_dataCapture.startIndex] = (uint8_t) (_raw.ayUnfiltered+_floatOrigin);
									//if it's not triggered, do nothing further
									//if it's triggered, set endIndex to startIndex+1
									int diffX = abs(int(_btn.Ax) - x0);
									int diffY = abs(int(_btn.Ay) - y0);
									if(diffX > _dataCapture.stickThresh || diffY > _dataCapture.stickThresh) {
										_dataCapture.endIndex = (_dataCapture.startIndex+1) % 100;
									}
								} else {
									_dataCapture.a1[_dataCapture.startIndex] = _btn.Cx;
									_dataCapture.a2[_dataCapture.startIndex] = _btn.Cy;
									_dataCapture.a1Unfilt[_dataCapture.startIndex] = (uint8_t) (_raw.cxUnfiltered+_floatOrigin);
									_dataCapture.a2Unfilt[_dataCapture.startIndex] = (uint8_t) (_raw.cyUnfiltered+_floatOrigin);
									//if it's not triggered, do nothing further
									//if it's triggered, set endIndex to startIndex+1
									int diffX = abs(int(_btn.Cx) - x0);
									int diffY = abs(int(_btn.Cy) - y0);
									if(diffX > _dataCapture.stickThresh || diffY > _dataCapture.stickThresh) {
										_dataCapture.endIndex = (_dataCapture.startIndex+1) % 100;
									}
								}
								//                                                                   syxba                       lrz
								_dataCapture.abxyszrl[_dataCapture.startIndex] = (_btn.arr[0] & 0b00011111) | ((_btn.arr[1] & 0b01110000) << 1);
							} else if(_dataCapture.endIndex != _dataCapture.startIndex) {
								//triggered
								//record
								if(_dataCapture.captureStick == ASTICK) {
									_dataCapture.a1[_dataCapture.endIndex] = _btn.Ax;
									_dataCapture.a2[_dataCapture.endIndex] = _btn.Ay;
									_dataCapture.a1Unfilt[_dataCapture.endIndex] = (uint8_t) (_raw.axUnfiltered+_floatOrigin);
									_dataCapture.a2Unfilt[_dataCapture.endIndex] = (uint8_t) (_raw.ayUnfiltered+_floatOrigin);
								} else {
									_dataCapture.a1[_dataCapture.endIndex] = _btn.Cx;
									_dataCapture.a2[_dataCapture.endIndex] = _btn.Cy;
									_dataCapture.a1Unfilt[_dataCapture.endIndex] = (uint8_t) (_raw.cxUnfiltered+_floatOrigin);
									_dataCapture.a2Unfilt[_dataCapture.endIndex] = (uint8_t) (_raw.cyUnfiltered+_floatOrigin);
								}
								//                                                                 syxba                       lrz
								_dataCapture.abxyszrl[_dataCapture.endIndex] = (_btn.arr[0] & 0b00011111) | ((_btn.arr[1] & 0b01110000) << 1);
								//increment endIndex
								_dataCapture.endIndex = (_dataCapture.endIndex+1) % 100;
							} else {
								//done
								_dataCapture.done = true;
								_pleaseCommit = 255;//end capture and display
							}
						}
						break;
					case CM_STICK_FALLING:
						//
						break;
					case CM_STICK_PIVOT:
						//
						break;
					case CM_TRIG:
						//
						break;
					case CM_JUMP:
						//
						break;
					case CM_PRESS:
						//
						break;
					default:
						//nothing
						break;
				}
			}
		}

		//gpio_put(_pinSpare0, !gpio_get_out_level(_pinSpare0));
		//pwm_set_gpio_level(_pinLED, 255*gpio_get_out_level(_pinSpare0));

		static bool running = false;

		//gpio_put(_pinSpare0, !gpio_get_out_level(_pinSpare0));
		//pwm_set_gpio_level(_pinLED, 255*gpio_get_out_level(_pinSpare0));

		//check if we should be reporting values yet
		if((_btn.B || _controls.autoInit || _videoOut) && !running){
			running=true;
		}

		//check to see if we are calibrating
		if(_currentCalStep >= 0){
			//Respond to inputs
			//Display and notch adjust stuff that needs to be updated every loop
			if(whichStick == ASTICK){
				if(_currentCalStep >= _noOfCalibrationPoints){//adjust notch angles
					adjustNotch(_currentCalStep, _dT, ASTICK, measuredNotchAngles, notchAngles, notchStatus, _btn, _hardware);
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
					displayNotch(_currentCalStep, true, _notchAngleDefaults, _btn);
				}
				readSticks(true,false, _btn, _pinList, _raw, _hardware, _controls, _normGains, _aStickParams, _cStickParams, _dT, _currentCalStep);
			}
			else{//WHICHSTICK == CSTICK
				if(_currentCalStep >= _noOfCalibrationPoints){//adjust notch angles
					adjustNotch(_currentCalStep, _dT, CSTICK, measuredNotchAngles, notchAngles, notchStatus, _btn, _hardware);
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
					displayNotch(_currentCalStep, false, _notchAngleDefaults, _btn);
				}
				readSticks(false,true, _btn, _pinList, _raw, _hardware, _controls, _normGains, _aStickParams, _cStickParams, _dT, _currentCalStep);
			}
		}
		else if(running){
			//if not calibrating read the sticks normally
			readSticks(true,true, _btn, _pinList, _raw, _hardware, _controls, _normGains, _aStickParams, _cStickParams, _dT, _currentCalStep);
		}

		//read the controller's buttons
		processButtons(_pinList, _btn, _hardware, _controls, _gains, _normGains, _currentCalStep, running, tempCalPointsX, tempCalPointsY, whichStick, notchStatus, notchAngles, measuredNotchAngles, _aStickParams, _cStickParams);

	}
}

int main() {
	//set the clock speed to 125 kHz
	//the comms library needs this clockspeed
	//it's actually the default so we don't need to
	//set_sys_clock_khz(1000*_us, true);

	//Read settings; true = don't lock out the core because running it on core 1 will freeze
	const int numberOfNaN = readEEPROM(_controls, _gains, _normGains, _aStickParams, _cStickParams, true);
	if(numberOfNaN > 10){//by default it seems 16 end up unitialized on pico
		resetDefaults(FACTORY, _controls, _gains, _normGains, _aStickParams, _cStickParams, true);
		readEEPROM(_controls, _gains, _normGains, _aStickParams, _cStickParams, true);
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

	_dataCapture.mode = CM_NULL;
	_dataCapture.triggerStick = ASTICK;
	_dataCapture.captureStick = ASTICK;
	_dataCapture.abxyszrlShow = 0;
	_dataCapture.begin = false;
	_dataCapture.done = true;
	_dataCapture.delay = 0;
	_dataCapture.stickThresh = 23;
	_dataCapture.triggerThresh = 49;
	_dataCapture.startIndex = 0;
	_dataCapture.endIndex = 0;

	//measure the trigger values for trigger tricking
	initializeButtons(_pinList, _btn, _controls.lTrigInitial, _controls.rTrigInitial);

	//Read buttons on startup to determine what mode to begin in
	readButtons(_pinList, _hardware);

	if(_hardware.S) { //hold start on powerup for BOOTSEL
		reset_usb_boot(0, 0);
	}

	if(_hardware.Z) { //hold Z on powerup for PhobVision
		_videoOut = true;
		set_sys_clock_khz(1000*250, true);//overclock to 250 khz, to alleviate performance issues
	}

	multicore_lockout_victim_init();

	multicore_launch_core1(second_core);

	//Run comms unless Z is held while plugging in
	if(_hardware.Z) {
#ifdef BUILD_DEV
		const int version = -SW_VERSION;
#else //BUILD_DEV
		const int version = SW_VERSION;
#endif //BUILD_DEV
		videoOut(_pinDac0, _btn, _hardware, _raw, _controls, _aStickParams, _cStickParams, _dataCapture, _sync, _pleaseCommit, _currentCalStep, version);
	} else {
		enterMode(_pinTX,
				_pinRumble,
				_pinBrake,
				_rumblePower,
				buttonsToGCReport);
	}

}
