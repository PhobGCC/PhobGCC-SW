//This software uses bits of code from GoodDoge's Dogebawx project, which was the initial starting point: https://github.com/DogeSSBM/DogeBawx

#include <math.h>
#include <EEPROM.h>
#include <ADC.h>
#include <VREF.h>

//Uncomment the appropriate #include line for your hardware by deleting the two slashes at the beginning of the line.
//#include "teensy/Phob1_0Teensy3_2.h"          // For PhobGCC board 1.0 with Teensy 3.2
//#include "teensy/Phob1_0Teensy3_2DiodeShort.h"// For PhobGCC board 1.0 with Teensy 3.2 and the diode shorted
//#include "teensy/Phob1_1Teensy3_2.h"          // For PhobGCC board 1.1 with Teensy 3.2
//#include "teensy/Phob1_1Teensy3_2DiodeShort.h"// For PhobGCC board 1.1 with Teensy 3.2 and the diode shorted
//#include "teensy/Phob1_1Teensy4_0.h"          // For PhobGCC board 1.1 with Teensy 4.0
//#include "teensy/Phob1_1Teensy4_0DiodeShort.h"// For PhobGCC board 1.1 with Teensy 4.0 and the diode shorted
#include "teensy/Phob1_2Teensy4_0.h"          // For PhobGCC board 1.2.x with Teensy 4.0

extern "C" uint32_t set_arm_clock(uint32_t frequency);

//index values to store data into eeprom
const int _bytesPerFloat = 4;
const int _eepromAPointsX = 0;
const int _eepromAPointsY = _eepromAPointsX+_noOfCalibrationPoints*_bytesPerFloat;
const int _eepromCPointsX = _eepromAPointsY+_noOfCalibrationPoints*_bytesPerFloat;
const int _eepromCPointsY = _eepromCPointsX+_noOfCalibrationPoints*_bytesPerFloat;
const int _eepromxSnapback = _eepromCPointsY+_noOfCalibrationPoints*_bytesPerFloat;
const int _eepromySnapback = _eepromxSnapback+_bytesPerFloat;
const int _eepromJump = _eepromySnapback+_bytesPerFloat;
const int _eepromANotchAngles = _eepromJump+_bytesPerFloat;
const int _eepromCNotchAngles = _eepromANotchAngles+_noOfNotches*_bytesPerFloat;
const int _eepromLToggle = _eepromCNotchAngles+_noOfNotches*_bytesPerFloat;
const int _eepromRToggle = _eepromLToggle+_bytesPerFloat;
const int _eepromcXOffset = _eepromRToggle+_bytesPerFloat;
const int _eepromcYOffset = _eepromcXOffset+_bytesPerFloat;
const int _eepromxSmoothing = _eepromcYOffset+_bytesPerFloat;
const int _eepromySmoothing = _eepromxSmoothing+_bytesPerFloat;
const int _eepromLOffset = _eepromySmoothing+_bytesPerFloat;
const int _eepromROffset = _eepromLOffset+_bytesPerFloat;
const int _eepromCxSmoothing = _eepromROffset+_bytesPerFloat;
const int _eepromCySmoothing = _eepromCxSmoothing+_bytesPerFloat;
const int _eepromRumble = _eepromCySmoothing+_bytesPerFloat;
const int _eepromAutoInit = _eepromRumble+_bytesPerFloat;


float _dT;
bool _running = false;




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

	const int numberOfNaN = readEEPROM(_controls, _gains, _normGains);
	Serial.print("Number of NaN in EEPROM: ");
	Serial.println(numberOfNaN);
	if(numberOfNaN > 3){//by default it seems 4 end up NaN on Teensy 4
		resetDefaults(HARD, _controls, _gains, _normGains);//do reset sticks
		readEEPROM(_controls, _gains, _normGains);
	}

	//set some of the unused values in the message response
	_btn.errS = 0;
	_btn.errL = 0;
	_btn.orig = 0;
	_btn.high = 1;

	_currentCalStep = _notCalibrating;

    setPinModes();

	_pinList.pinLa = _pinLa;
	_pinList.pinRa = _pinRa;
	_pinList.pinL  = _pinL;
	_pinList.pinR  = _pinR;
	_pinList.pinAx = _pinAx;
	_pinList.pinAy = _pinAy;
	_pinList.pinCx = _pinCx;
	_pinList.pinCy = _pinCy;
	_pinList.pinRX = _pinRX;
	_pinList.pinTX = _pinTX;
	_pinList.pinDr = _pinDr;
	_pinList.pinDu = _pinDu;
	_pinList.pinDl = _pinDl;
	_pinList.pinDd = _pinDd;
	_pinList.pinX  = _pinX;
	_pinList.pinY  = _pinY;
	_pinList.pinA  = _pinA;
	_pinList.pinB  = _pinB;
	_pinList.pinZ  = _pinZ;
	_pinList.pinS  = _pinS;

    ADCSetup(adc, _ADCScale, _ADCScaleFactor);

	//measure the trigger values
	initializeButtons(_btn,_controls.lTrigInitial,_controls.rTrigInitial);
	//set the origin response before the sticks have been touched
	//it will never be changed again after this

	commsSetup(_btn);
}

void loop() {
	//check if we should be reporting values yet
	if((_btn.B || _controls.autoInit) && !_running){
		Serial.println("Starting to report values");
		_running=true;
	}

	//read the controllers buttons
	readButtons(_btn, _hardware, _controls, _gains, _normGains);

	//check to see if we are calibrating
	if(_currentCalStep >= 0){
		if(_calAStick){
			if(_currentCalStep >= _noOfCalibrationPoints){//adjust notch angles
				adjustNotch(_currentCalStep, _dT, true, _measuredNotchAngles, _aNotchAngles, _aNotchStatus, _btn, _hardware);
				if(_hardware.Y || _hardware.X || (_btn.B)){//only run this if the notch was adjusted
					//clean full cal points again, feeding updated angles in
					cleanCalPoints(_tempCalPointsX, _tempCalPointsY, _aNotchAngles, _cleanedPointsX, _cleanedPointsY, _notchPointsX, _notchPointsY, _aNotchStatus);
					//linearize again
					linearizeCal(_cleanedPointsX, _cleanedPointsY, _cleanedPointsX, _cleanedPointsY, _aFitCoeffsX, _aFitCoeffsY);
					//notchCalibrate again to update the affine transform
					notchCalibrate(_cleanedPointsX, _cleanedPointsY, _notchPointsX, _notchPointsY, _noOfNotches, _aAffineCoeffs, _aBoundaryAngles);
				}
			}else{//just show desired stick position
				displayNotch(_currentCalStep, true, _notchAngleDefaults, _btn);
			}
			readSticks(true,false, _btn, _pinList, _hardware, _controls, _normGains, _dT);
		}
		else{
			if(_currentCalStep >= _noOfCalibrationPoints){//adjust notch angles
				adjustNotch(_currentCalStep, _dT, false, _measuredNotchAngles, _cNotchAngles, _cNotchStatus, _btn, _hardware);
				if(_hardware.Y || _hardware.X || (_btn.B)){//only run this if the notch was adjusted
					//clean full cal points again, feeding updated angles in
					cleanCalPoints(_tempCalPointsX, _tempCalPointsY, _cNotchAngles, _cleanedPointsX, _cleanedPointsY, _notchPointsX, _notchPointsY, _cNotchStatus);
					//linearize again
					linearizeCal(_cleanedPointsX, _cleanedPointsY, _cleanedPointsX, _cleanedPointsY, _cFitCoeffsX, _cFitCoeffsY);
					//notchCalibrate again to update the affine transform
					notchCalibrate(_cleanedPointsX, _cleanedPointsY, _notchPointsX, _notchPointsY, _noOfNotches, _cAffineCoeffs, _cBoundaryAngles);
				}
			}else{//just show desired stick position
				displayNotch(_currentCalStep, false, _notchAngleDefaults, _btn);
			}
			readSticks(false,true, _btn, _pinList, _hardware, _controls, _normGains, _dT);
		}
	}
	else if(_running){
		//if not calibrating read the sticks normally
		readSticks(true,true, _btn, _pinList, _hardware, _controls, _normGains, _dT);
	}
}


int readEEPROM(ControlConfig &controls, FilterGains &gains, FilterGains &normGains){
	int numberOfNaN = 0;

	//get the jump setting
	//EEPROM.get(_eepromJump, controls.jumpConfig);
	controls.jumpConfig = getJumpSetting();
	if(controls.jumpConfig < controls.jumpConfigMin){
		controls.jumpConfig = DEFAULTJUMP;
		numberOfNaN++;
	}
	if(controls.jumpConfig > controls.jumpConfigMax){
		controls.jumpConfig = DEFAULTJUMP;
		numberOfNaN++;
	}
	setJump(controls);

	//get the L setting
	//EEPROM.get(_eepromLToggle, controls.lConfig);
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
	//EEPROM.get(_eepromRToggle, controls.rConfig);
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
	//EEPROM.get(_eepromLOffset, controls.lTriggerOffset);
	controls.lTriggerOffset = getLOffsetSetting();
	if(controls.lTriggerOffset > controls.triggerMax) {
		controls.lTriggerOffset = controls.triggerMax;
		numberOfNaN++;
	} else if(controls.lTriggerOffset < controls.triggerMin) {
		controls.lTriggerOffset = controls.triggerMin;
		numberOfNaN++;
	}

	//get the R-trigger Offset value
	//EEPROM.get(_eepromROffset, controls.rTriggerOffset);
	controls.rTriggerOffset = getROffsetSetting();
	if(controls.rTriggerOffset > controls.triggerMax) {
		controls.rTriggerOffset = controls.triggerMax;
		numberOfNaN++;
	} else if(controls.rTriggerOffset < controls.triggerMin) {
		controls.rTriggerOffset = controls.triggerMin;
		numberOfNaN++;
	}


	//get the C-stick X offset
	//EEPROM.get(_eepromcXOffset, controls.cXOffset);
	controls.cXOffset = getCXOffsetSetting();
	if(controls.cXOffset > controls.cMax) {
		controls.cXOffset = 0;
		numberOfNaN++;
	} else if(controls.cXOffset < controls.cMin) {
		controls.cXOffset = 0;
		numberOfNaN++;
	}

	//get the C-stick Y offset
	//EEPROM.get(_eepromcYOffset, controls.cYOffset);
	controls.cYOffset = getCYOffsetSetting();
	if(controls.cYOffset > controls.cMax) {
		controls.cYOffset = 0;
		numberOfNaN++;
	} else if(controls.cYOffset < controls.cMin) {
		controls.cYOffset = 0;
		numberOfNaN++;
	}

	//get the x-axis snapback correction
	EEPROM.get(_eepromxSnapback, controls.xSnapback);
	Serial.print("the xSnapback value from eeprom is:");
	Serial.println(controls.xSnapback);
	if(controls.xSnapback < controls.snapbackMin) {
		controls.xSnapback = controls.snapbackMin;
		numberOfNaN++;
	} else if (controls.xSnapback > controls.snapbackMax) {
		controls.xSnapback = controls.snapbackMax;
		numberOfNaN++;
	}
	gains.xVelDamp = velDampFromSnapback(controls.xSnapback);
	Serial.print("the xVelDamp value from eeprom is:");
	Serial.println(gains.xVelDamp);

	//get the y-ayis snapback correction
	EEPROM.get(_eepromySnapback, controls.ySnapback);
	Serial.print("the ySnapback value from eeprom is:");
	Serial.println(controls.ySnapback);
	if(controls.ySnapback < controls.snapbackMin) {
		controls.ySnapback = controls.snapbackMin;
		numberOfNaN++;
	} else if (controls.ySnapback > controls.snapbackMax) {
		controls.ySnapback = controls.snapbackMax;
		numberOfNaN++;
	}
	gains.yVelDamp = velDampFromSnapback(controls.ySnapback);
	Serial.print("the yVelDamp value from eeprom is:");
	Serial.println(gains.yVelDamp);

	//get the x-axis smoothing value
	EEPROM.get(_eepromxSmoothing, gains.xSmoothing);
	Serial.print("the xSmoothing value from eeprom is:");
	Serial.println(gains.xSmoothing);
	if(std::isnan(gains.xSmoothing)){
		gains.xSmoothing = controls.smoothingMin;
		Serial.print("the xSmoothing value was adjusted to:");
		Serial.println(gains.xSmoothing);
		numberOfNaN++;
	}
	if(gains.xSmoothing > controls.smoothingMax) {
		gains.xSmoothing = controls.smoothingMax;
	} else if(gains.xSmoothing < controls.smoothingMin) {
		gains.xSmoothing = controls.smoothingMin;
	}

	//get the y-axis smoothing value
	EEPROM.get(_eepromySmoothing, gains.ySmoothing);
	Serial.print("the ySmoothing value from eeprom is:");
	Serial.println(gains.ySmoothing);
	if(std::isnan(gains.ySmoothing)){
		gains.ySmoothing = controls.smoothingMin;
		Serial.print("the ySmoothing value was adjusted to:");
		Serial.println(gains.ySmoothing);
		numberOfNaN++;
	}
	if(gains.ySmoothing > controls.smoothingMax) {
		gains.ySmoothing = controls.smoothingMax;
	} else if(gains.ySmoothing < controls.smoothingMin) {
		gains.ySmoothing = controls.smoothingMin;
	}

	//get the c-stick x-axis smoothing value
	EEPROM.get(_eepromCxSmoothing, gains.cXSmoothing);
	Serial.print("the cXSmoothing value from eeprom is:");
	Serial.println(gains.cXSmoothing);
	if(std::isnan(gains.cXSmoothing)){
		gains.cXSmoothing = controls.smoothingMin;
		Serial.print("the cXSmoothing value was adjusted to:");
		Serial.println(gains.cXSmoothing);
		numberOfNaN++;
	}
	if(gains.cXSmoothing > controls.smoothingMax) {
		gains.cXSmoothing = controls.smoothingMax;
	} else if(gains.cXSmoothing < controls.smoothingMin) {
		gains.cXSmoothing = controls.smoothingMin;
	}

	//get the c-stick y-axis smoothing value
	EEPROM.get(_eepromCySmoothing, gains.cYSmoothing);
	Serial.print("the cYSmoothing value from eeprom is:");
	Serial.println(gains.cYSmoothing);
	if(std::isnan(gains.cYSmoothing)){
		gains.cYSmoothing = controls.smoothingMin;
		Serial.print("the cYSmoothing value was adjusted to:");
		Serial.println(gains.cYSmoothing);
		numberOfNaN++;
	}
	if(gains.cYSmoothing > controls.smoothingMax) {
		gains.cYSmoothing = controls.smoothingMax;
	} else if(gains.cYSmoothing < controls.smoothingMin) {
		gains.cYSmoothing = controls.smoothingMin;
	}

	//recompute the intermediate gains used directly by the kalman filter
	recomputeGains(gains, normGains);

	//Get the rumble value
	EEPROM.get(_eepromRumble, controls.rumble);
	Serial.print("Rumble value before fixing: ");
	Serial.println(controls.rumble);
	if(std::isnan(controls.rumble)) {
		controls.rumble = controls.rumbleDefault;
		numberOfNaN++;
	}
	if(controls.rumble < controls.rumbleMin) {
		controls.rumble = controls.rumbleMin;
	}
	if(controls.rumble > controls.rumbleMax) {
		controls.rumble = controls.rumbleMax;
	}
	_rumblePower = calcRumblePower(controls.rumble);
	Serial.print("Rumble value: ");
	Serial.println(controls.rumble);
	Serial.print("Rumble power: ");
	Serial.println(_rumblePower);

	//Get the autoinit value
	EEPROM.get(_eepromAutoInit, controls.autoInit);
	if(controls.autoInit < 0) {
		controls.autoInit = 0;
		numberOfNaN++;
	}
	if(controls.autoInit > 1) {
		controls.autoInit = 0;
		numberOfNaN++;
	}
	Serial.print("Auto init: ");
	Serial.println(controls.autoInit);

	//get the calibration points collected during the last A stick calibration
	EEPROM.get(_eepromAPointsX, _tempCalPointsX);
	EEPROM.get(_eepromAPointsY, _tempCalPointsY);
	EEPROM.get(_eepromANotchAngles, _aNotchAngles);
	cleanCalPoints(_tempCalPointsX, _tempCalPointsY, _aNotchAngles, _cleanedPointsX, _cleanedPointsY, _notchPointsX, _notchPointsY, _aNotchStatus);
	Serial.println("calibration points cleaned");
	linearizeCal(_cleanedPointsX, _cleanedPointsY, _cleanedPointsX, _cleanedPointsY, _aFitCoeffsX, _aFitCoeffsY);
	Serial.println("A stick linearized");
	notchCalibrate(_cleanedPointsX, _cleanedPointsY, _notchPointsX, _notchPointsY, _noOfNotches, _aAffineCoeffs, _aBoundaryAngles);
	//stickCal(_cleanedPointsX,_cleanedPointsY,_aNotchAngles,_aFitCoeffsX,_aFitCoeffsY,_aAffineCoeffs,_aBoundaryAngles);

	//get the calibration points collected during the last A stick calibration
	EEPROM.get(_eepromCPointsX, _tempCalPointsX);
	EEPROM.get(_eepromCPointsY, _tempCalPointsY);
	EEPROM.get(_eepromCNotchAngles, _cNotchAngles);
	cleanCalPoints(_tempCalPointsX, _tempCalPointsY, _cNotchAngles, _cleanedPointsX, _cleanedPointsY, _notchPointsX, _notchPointsY, _cNotchStatus);
	Serial.println("calibration points cleaned");
	linearizeCal(_cleanedPointsX, _cleanedPointsY, _cleanedPointsX, _cleanedPointsY, _cFitCoeffsX, _cFitCoeffsY);
	Serial.println("C stick linearized");
	notchCalibrate(_cleanedPointsX, _cleanedPointsY, _notchPointsX, _notchPointsY, _noOfNotches, _cAffineCoeffs, _cBoundaryAngles);
	//stickCal(_cleanedPointsX,_cleanedPointsY,_cNotchAngles,_cFitCoeffsX,_cFitCoeffsY,_cAffineCoeffs,_cBoundaryAngles);

	return numberOfNaN;
}

void resetDefaults(HardReset reset, ControlConfig &controls, FilterGains &gains, FilterGains &normGains){
	Serial.println("RESETTING ALL DEFAULTS");

	controls.jumpConfig = DEFAULTJUMP;
	setJump(controls);
	//EEPROM.put(_eepromJump,controls.jumpConfig);
	setJumpSetting(controls.jumpConfig);

	controls.lConfig = controls.triggerDefault;
	controls.rConfig = controls.triggerDefault;
	//EEPROM.put(_eepromLToggle, controls.lConfig);
	setLSetting(controls.lConfig);
	//EEPROM.put(_eepromRToggle, controls.rConfig);
	setRSetting(controls.rConfig);

	controls.cXOffset = 0;
	controls.cYOffset = 0;
	//EEPROM.put(_eepromcXOffset, controls.cXOffset);
	setCXOffsetSetting(controls.cXOffset);
	//EEPROM.put(_eepromcYOffset, controls.cYOffset);
	setCYOffsetSetting(controls.cYOffset);

	controls.xSnapback = controls.snapbackDefault;
	EEPROM.put(_eepromxSnapback,controls.xSnapback);
	gains.xVelDamp = velDampFromSnapback(controls.xSnapback);
	controls.ySnapback = controls.snapbackDefault;
	EEPROM.put(_eepromySnapback,controls.ySnapback);
	gains.yVelDamp = velDampFromSnapback(controls.ySnapback);

	gains.xSmoothing = controls.smoothingMin;
	EEPROM.put(_eepromxSmoothing, gains.xSmoothing);
	gains.ySmoothing = controls.smoothingMin;
	EEPROM.put(_eepromySmoothing, gains.ySmoothing);

	gains.cXSmoothing = controls.smoothingMin;
	EEPROM.put(_eepromCxSmoothing, gains.cXSmoothing);
	gains.cYSmoothing = controls.smoothingMin;
	EEPROM.put(_eepromCySmoothing, gains.cYSmoothing);
	//recompute the intermediate gains used directly by the kalman filter
	recomputeGains(gains, normGains);

	controls.lTriggerOffset = controls.triggerMin;
	controls.rTriggerOffset = controls.triggerMin;
	//EEPROM.put(_eepromLOffset, controls.lTriggerOffset);
	setLOffsetSetting(controls.lTriggerOffset);
	//EEPROM.put(_eepromROffset, controls.rTriggerOffset);
	setROffsetSetting(controls.rTriggerOffset);

	controls.rumble = controls.rumbleDefault;
	_rumblePower = calcRumblePower(controls.rumble);
	EEPROM.put(_eepromRumble, controls.rumble);

	//always cancel auto init on reset, even if we don't reset the sticks
	controls.autoInit = 0;
	EEPROM.put(_eepromAutoInit, controls.autoInit);

	if(reset == HARD){
		for(int i = 0; i < _noOfNotches; i++){
			_aNotchAngles[i] = _notchAngleDefaults[i];
			_cNotchAngles[i] = _notchAngleDefaults[i];
		}
		EEPROM.put(_eepromANotchAngles,_aNotchAngles);
		EEPROM.put(_eepromCNotchAngles,_cNotchAngles);

		for(int i = 0; i < _noOfCalibrationPoints; i++){
			_tempCalPointsX[i] = _defaultCalPointsX[i];
			_tempCalPointsY[i] = _defaultCalPointsY[i];
		}
		EEPROM.put(_eepromAPointsX,_tempCalPointsX);
		EEPROM.put(_eepromAPointsY,_tempCalPointsY);

		Serial.println("A calibration points stored in EEPROM");
		cleanCalPoints(_tempCalPointsX, _tempCalPointsY, _aNotchAngles, _cleanedPointsX, _cleanedPointsY, _notchPointsX, _notchPointsY, _aNotchStatus);
		Serial.println("A calibration points cleaned");
		linearizeCal(_cleanedPointsX, _cleanedPointsY, _cleanedPointsX, _cleanedPointsY, _aFitCoeffsX, _aFitCoeffsY);
		Serial.println("A stick linearized");
		notchCalibrate(_cleanedPointsX, _cleanedPointsY, _notchPointsX, _notchPointsY, _noOfNotches, _aAffineCoeffs, _aBoundaryAngles);

		for(int i = 0; i < _noOfCalibrationPoints; i++){
			_tempCalPointsX[i] = _defaultCalPointsX[i];
			_tempCalPointsY[i] = _defaultCalPointsY[i];
		}
		EEPROM.put(_eepromCPointsX,_tempCalPointsX);
		EEPROM.put(_eepromCPointsY,_tempCalPointsY);

		Serial.println("C calibration points stored in EEPROM");
		cleanCalPoints(_tempCalPointsX, _tempCalPointsY, _cNotchAngles, _cleanedPointsX, _cleanedPointsY, _notchPointsX, _notchPointsY, _cNotchStatus);
		Serial.println("C calibration points cleaned");
		linearizeCal(_cleanedPointsX, _cleanedPointsY, _cleanedPointsX, _cleanedPointsY, _cFitCoeffsX, _cFitCoeffsY);
		Serial.println("C stick linearized");
		notchCalibrate(_cleanedPointsX, _cleanedPointsY, _notchPointsX, _notchPointsY, _noOfNotches, _cAffineCoeffs, _cBoundaryAngles);
	}
}
void setPinModes(){
	pinMode(_pinL,INPUT_PULLUP);
	pinMode(_pinR,INPUT_PULLUP);
	pinMode(_pinDr,INPUT_PULLUP);
	pinMode(_pinDu,INPUT_PULLUP);
	pinMode(_pinDl,INPUT_PULLUP);
	pinMode(_pinDd,INPUT_PULLUP);
	pinMode(_pinX,INPUT_PULLUP);
	pinMode(_pinY,INPUT_PULLUP);

	pinMode(_pinA,INPUT_PULLUP);
	pinMode(_pinB,INPUT_PULLUP);
	pinMode(_pinZ,INPUT_PULLUP);
	pinMode(_pinS,INPUT_PULLUP);
#ifdef TEENSY4_0
#ifdef HALFDUPLEX
	pinMode(_pinRX,INPUT_PULLUP);
#else // HALFDUPLEX
	pinMode(9,    INPUT_PULLUP); //the normal RX pin doesn't work on teensy 4 with full duplex
#endif // HALFDUPLEX
	pinMode(_pinLED,   OUTPUT);
#endif // TEENSY4_0

#ifdef RUMBLE
	pinMode(_pinRumble, OUTPUT);
	pinMode(_pinBrake, OUTPUT);
#endif

	//Teensy 4 has some weird jump in the analog with default pin mode
	pinMode(_pinLa,INPUT_DISABLE);
	pinMode(_pinRa,INPUT_DISABLE);
	pinMode(_pinAx,INPUT_DISABLE);
	pinMode(_pinAy,INPUT_DISABLE);
	pinMode(_pinCx,INPUT_DISABLE);
	pinMode(_pinCy,INPUT_DISABLE);
}
void readButtons(Buttons &btn, HardwareButtons &hardware, ControlConfig &controls, FilterGains &gains, FilterGains &normGains){
	btn.A = !digitalRead(_pinA);
	btn.B = !digitalRead(_pinB);
	btn.X = !digitalRead(controls.pinXSwappable);
	btn.Y = !digitalRead(controls.pinYSwappable);
	btn.Z = !digitalRead(controls.pinZSwappable);
	btn.S = !digitalRead(_pinS);
	btn.Du = !digitalRead(_pinDu);
	btn.Dd = !digitalRead(_pinDd);
	btn.Dl = !digitalRead(_pinDl);
	btn.Dr = !digitalRead(_pinDr);

	hardware.L = !digitalRead(_pinL);
	hardware.R = !digitalRead(_pinR);
	hardware.Z = !digitalRead(_pinZ);
	hardware.X = !digitalRead(_pinX);
	hardware.Y = !digitalRead(_pinY);

	//We apply the triggers in readSticks so we can minimize race conditions
	// between trigger analog/digital so we don't get ADT vulnerability in mode 6

	/* Current Commands List
	* Safe Mode:  AXY+Start
	* Display Version: AZ+Du
	*
	* Soft Reset:  ABZ+Start
	* Hard Reset:  ABZ+Dd
	* Auto-Initialize: ABLR+Start
	*
	* Increase/Decrease Rumble: XY+Du/Dd
	* Show Current Rumble Setting: BXY (no A)
	*
	* Calibration
	* Analog Stick Calibration:  AXY+L
	* C-Stick Calibration:  AXY+R
	* Advance Calibration:  L or R
	* Undo Calibration:  Z
	* Skip to Notch Adjustment:  Start
	* Notch Adjustment CW/CCW:  X/Y
	* Notch Adjustment Reset:  B
	*
	* Analog Stick Configuration:
	* Increase/Decrease X-Axis Snapback Filtering:  LX+Du/Dd
	* Increase/Decrease Y-Axis Snapback Filtering:  LY+Du/Dd
	* Increase/Decrease X-Axis Delay:  LA+Du/Dd
	* Increase/Decrease Y-Axis Delay:  LB+Du/Dd
	* Show Filtering and Axis Delay:  LStart+Dd
	*
	* C-Stick Configuration
	* Increase/Decrease X-Axis Snapback Filtering:  RX+Du/Dd
	* Increase/Decrease Y-Axis Snapback Filtering:  RY+Du/Dd
	* Increase/Decrease X-Axis Offset:  RA+Du/Dd
	* Increase/Decrease Y-Axis Offset:  RB+Du/Dd
	* Show Filtering and Axis Offset:  RStart+Dd
	*
	* Swap X with Z:  XZ+Start
	* Swap Y with Z:  YZ+Start
	* Reset Z-Jump:  AXY+Z
	* Toggle Analog Slider L:  ZL+Start
	* Toggle Analog Slider R:  ZR+Start
	* Increase/Decrease L-trigger Offset:  ZL+Du/Dd
	* Increase/Decrease R-Trigger Offset:  ZR+Du/Dd
	*/

	//check the dpad buttons to change the controller settings
	if(!controls.safeMode && (_currentCalStep == -1)) {
		if(btn.A && hardware.X && hardware.Y && btn.S) { //Safe Mode Toggle
			controls.safeMode = true;
			freezeSticks(4000, btn, hardware);
		} else if (btn.A && hardware.Z && btn.Du) { //display version number
			const int versionHundreds = floor(SW_VERSION/100.0);
			const int versionOnes     = SW_VERSION-versionHundreds;
			btn.Ax = (uint8_t) 127.5;
			btn.Ay = (uint8_t) 127.5;
			btn.Cx = (uint8_t) 127.5 + versionHundreds;
			btn.Cy = (uint8_t) 127.5 + versionOnes;
			clearButtons(2000, btn, hardware);
		} else if (btn.A && btn.B && hardware.Z && btn.S) { //Soft Reset
			resetDefaults(SOFT, controls, gains, normGains);//don't reset sticks
			freezeSticks(2000, btn, hardware);
		} else if (btn.A && btn.B && hardware.Z && btn.Dd) { //Hard Reset
			resetDefaults(HARD, controls, gains, normGains);//do reset sticks
			freezeSticks(2000, btn, hardware);
		} else if (btn.A && btn.B && hardware.L && hardware.R && btn.S) { //Toggle Auto-Initialize
			changeAutoInit(btn, hardware, controls);
		} else if (hardware.X && hardware.Y && btn.Du) { //Increase Rumble
#ifdef RUMBLE
			changeRumble(INCREASE, btn, hardware, controls);
#else // RUMBLE
			//nothing
			freezeSticks(2000, btn, hardware);
#endif // RUMBLE
		} else if (hardware.X && hardware.Y && btn.Dd) { //Decrease Rumble
#ifdef RUMBLE
			changeRumble(DECREASE, btn, hardware, controls);
#else // RUMBLE
			//nothing
			freezeSticks(2000, btn, hardware);
#endif // RUMBLE
		} else if (hardware.X && hardware.Y && btn.B && !btn.A) { //Show current rumble setting
#ifdef RUMBLE
			showRumble(2000, btn, hardware, controls);
#else // RUMBLE
			freezeSticks(2000, btn, hardware);
#endif // RUMBLE
		} else if (btn.A && hardware.X && hardware.Y && hardware.L) { //Analog Calibration
			Serial.println("Calibrating the A stick");
			_calAStick = true;
			_currentCalStep ++;
			_advanceCal = true;
			freezeSticks(2000, btn, hardware);
		} else if (btn.A && hardware.X && hardware.Y && hardware.R) { //C-stick Calibration
			Serial.println("Calibrating the C stick");
			_calAStick = false;
			_currentCalStep ++;
			_advanceCal = true;
			freezeSticks(2000, btn, hardware);
		} else if(hardware.L && hardware.X && btn.Du) { //Increase Analog X-Axis Snapback Filtering
			adjustSnapback(XAXIS, INCREASE, btn, hardware, controls, gains, normGains);
		} else if(hardware.L && hardware.X && btn.Dd) { //Decrease Analog X-Axis Snapback Filtering
			adjustSnapback(XAXIS, DECREASE, btn, hardware, controls, gains, normGains);
		} else if(hardware.L && hardware.Y && btn.Du) { //Increase Analog Y-Axis Snapback Filtering
			adjustSnapback(YAXIS, INCREASE, btn, hardware, controls, gains, normGains);
		} else if(hardware.L && hardware.Y && btn.Dd) { //Decrease Analog Y-Axis Snapback Filtering
			adjustSnapback(YAXIS, DECREASE, btn, hardware, controls, gains, normGains);
		} else if(hardware.L && btn.A && btn.Du) { //Increase X-axis Delay
			adjustSmoothing(XAXIS, INCREASE, btn, hardware, controls, gains, normGains);
		} else if(hardware.L && btn.A && btn.Dd) { //Decrease X-axis Delay
			adjustSmoothing(XAXIS, DECREASE, btn, hardware, controls, gains, normGains);
		} else if(hardware.L && btn.B && btn.Du) { //Increase Y-axis Delay
			adjustSmoothing(YAXIS, INCREASE, btn, hardware, controls, gains, normGains);
		} else if(hardware.L && btn.B && btn.Dd) { //Decrease Y-axis Delay
			adjustSmoothing(YAXIS, DECREASE, btn, hardware, controls, gains, normGains);
		} else if(hardware.L && btn.S && btn.Dd) { //Show Current Analog Settings
			showAstickSettings(btn, hardware, controls, gains);
		} else if(hardware.R && hardware.X && btn.Du) { //Increase C-stick X-Axis Snapback Filtering
			adjustCstickSmoothing(XAXIS, INCREASE, btn, hardware, controls, gains, normGains);
		} else if(hardware.R && hardware.X && btn.Dd) { //Decrease C-stick X-Axis Snapback Filtering
			adjustCstickSmoothing(XAXIS, DECREASE, btn, hardware, controls, gains, normGains);
		} else if(hardware.R && hardware.Y && btn.Du) { //Increase C-stick Y-Axis Snapback Filtering
			adjustCstickSmoothing(YAXIS, INCREASE, btn, hardware, controls, gains, normGains);
		} else if(hardware.R && hardware.Y && btn.Dd) { //Decrease C-stick Y-Axis Snapback Filtering
			adjustCstickSmoothing(YAXIS, DECREASE, btn, hardware, controls, gains, normGains);
		} else if(hardware.R && btn.A && btn.Du) { //Increase C-stick X Offset
			adjustCstickOffset(XAXIS, INCREASE, btn, hardware, controls);
		} else if(hardware.R && btn.A && btn.Dd) { //Decrease C-stick X Offset
			adjustCstickOffset(XAXIS, DECREASE, btn, hardware, controls);
		} else if(hardware.R && btn.B && btn.Du) { //Increase C-stick Y Offset
			adjustCstickOffset(YAXIS, INCREASE, btn, hardware, controls);
		} else if(hardware.R && btn.B && btn.Dd) { //Decrease C-stick Y Offset
			adjustCstickOffset(YAXIS, DECREASE, btn, hardware, controls);
		} else if(hardware.R && btn.S && btn.Dd) { //Show Current C-stick SEttings
			showCstickSettings(btn, hardware, controls, gains);
		} else if(hardware.L && hardware.Z && btn.S) { //Toggle Analog L
			nextTriggerState(LTRIGGER, btn, hardware, controls);
		} else if(hardware.R && hardware.Z && btn.S) { //Toggle Analog R
			nextTriggerState(RTRIGGER, btn, hardware, controls);
		} else if(hardware.L && hardware.Z && btn.Du) { //Increase L-Trigger Offset
			adjustTriggerOffset(LTRIGGER, INCREASE, btn, hardware, controls);
		} else if(hardware.L && hardware.Z && btn.Dd) { //Decrease L-trigger Offset
			adjustTriggerOffset(LTRIGGER, DECREASE, btn, hardware, controls);
		} else if(hardware.R && hardware.Z && btn.Du) { //Increase R-trigger Offset
			adjustTriggerOffset(RTRIGGER, INCREASE, btn, hardware, controls);
		} else if(hardware.R && hardware.Z && btn.Dd) { //Decrease R-trigger Offset
			adjustTriggerOffset(RTRIGGER, DECREASE, btn, hardware, controls);
		} else if(hardware.X && hardware.Z && btn.S) { //Swap X and Z
			readJumpConfig(SWAP_XZ, controls);
			freezeSticks(2000, btn, hardware);
		} else if(hardware.Y && hardware.Z && btn.S) { //Swap Y and Z
			readJumpConfig(SWAP_YZ, controls);
			freezeSticks(2000, btn, hardware);
		} else if(btn.A && hardware.X && hardware.Y && hardware.Z) { // Reset X/Y/Z Config
			readJumpConfig(DEFAULTJUMP, controls);
			freezeSticks(2000, btn, hardware);
		}
	} else if (_currentCalStep == -1) { //Safe Mode Enabled, Lock Settings, wait for safe mode command
		static float safeModeAccumulator = 0.0;
		if(btn.A && hardware.X && hardware.Y && btn.S) { //Safe Mode Toggle
			safeModeAccumulator = 0.99*safeModeAccumulator + 0.01;
		} else {
			safeModeAccumulator = 0.99*safeModeAccumulator;
		}
		if(safeModeAccumulator > 0.99){
			safeModeAccumulator = 0;
			if (!_running) {//wake it up if not already running
				_running = true;
			}
			controls.safeMode = false;
			freezeSticks(2000, btn, hardware);
		}
	}

	//Skip stick measurement and go to notch adjust using the start button while calibrating
	if(btn.S && (_currentCalStep >= 0 && _currentCalStep < 32)){
		_currentCalStep = _noOfCalibrationPoints;
		//Do the same thing we would have done at step 32 had we actually collected the points, but with stored tempCalPoints
		if(!_calAStick){
			//get the calibration points collected during the last A stick calibration
			EEPROM.get(_eepromCPointsX, _tempCalPointsX);
			EEPROM.get(_eepromCPointsY, _tempCalPointsY);
			EEPROM.get(_eepromCNotchAngles, _cNotchAngles);
			//make temp temp cal points that are missing all tertiary notches so that we get a neutral grid
			float tempCalPointsX[_noOfCalibrationPoints];
			float tempCalPointsY[_noOfCalibrationPoints];
			stripCalPoints(_tempCalPointsX, _tempCalPointsY, tempCalPointsX, tempCalPointsY);
			//clean the stripped calibration points, use default angles
			cleanCalPoints(tempCalPointsX, tempCalPointsY, _notchAngleDefaults, _cleanedPointsX, _cleanedPointsY, _notchPointsX, _notchPointsY, _cNotchStatus);
			linearizeCal(_cleanedPointsX, _cleanedPointsY, _cleanedPointsX, _cleanedPointsY, _cFitCoeffsX, _cFitCoeffsY);
			notchCalibrate(_cleanedPointsX, _cleanedPointsY, _notchPointsX, _notchPointsY, _noOfNotches, _cAffineCoeffs, _cBoundaryAngles);
			//apply the calibration to the original measured values including any tertiaries; we don't care about the angles
			cleanCalPoints(_tempCalPointsX, _tempCalPointsY, _notchAngleDefaults, _cleanedPointsX, _cleanedPointsY, _notchPointsX, _notchPointsY, _cNotchStatus);
			float transformedX[_noOfNotches+1];
			float transformedY[_noOfNotches+1];
			transformCalPoints(_cleanedPointsX, _cleanedPointsY, transformedX, transformedY, _cFitCoeffsX, _cFitCoeffsY, _cAffineCoeffs, _cBoundaryAngles);
			//compute the angles for those notches into _measuredNotchAngles, using the default angles for the diagonals
			computeStickAngles(transformedX, transformedY, _measuredNotchAngles);
			//clean full cal points again, feeding those angles in
			cleanCalPoints(_tempCalPointsX, _tempCalPointsY, _measuredNotchAngles, _cleanedPointsX, _cleanedPointsY, _notchPointsX, _notchPointsY, _cNotchStatus);
			//clear unused notch angles
			cleanNotches(_cNotchAngles, _measuredNotchAngles, _cNotchStatus);
			//clean full cal points again again, feeding those measured angles in for missing tertiary notches
			cleanCalPoints(_tempCalPointsX, _tempCalPointsY, _cNotchAngles, _cleanedPointsX, _cleanedPointsY, _notchPointsX, _notchPointsY, _cNotchStatus);
			//linearize again
			linearizeCal(_cleanedPointsX, _cleanedPointsY, _cleanedPointsX, _cleanedPointsY, _cFitCoeffsX, _cFitCoeffsY);
			//notchCalibrate again
			notchCalibrate(_cleanedPointsX, _cleanedPointsY, _notchPointsX, _notchPointsY, _noOfNotches, _cAffineCoeffs, _cBoundaryAngles);
		} else if(_calAStick){
			//get the calibration points collected during the last A stick calibration
			EEPROM.get(_eepromAPointsX, _tempCalPointsX);
			EEPROM.get(_eepromAPointsY, _tempCalPointsY);
			EEPROM.get(_eepromANotchAngles, _aNotchAngles);
			//make temp temp cal points that are missing all tertiary notches so that we get a neutral grid
			float tempCalPointsX[_noOfCalibrationPoints];
			float tempCalPointsY[_noOfCalibrationPoints];
			stripCalPoints(_tempCalPointsX, _tempCalPointsY, tempCalPointsX, tempCalPointsY);
			//clean the stripped calibration points, use default angles
			cleanCalPoints(tempCalPointsX, tempCalPointsY, _notchAngleDefaults, _cleanedPointsX, _cleanedPointsY, _notchPointsX, _notchPointsY, _aNotchStatus);
			linearizeCal(_cleanedPointsX, _cleanedPointsY, _cleanedPointsX, _cleanedPointsY, _aFitCoeffsX, _aFitCoeffsY);
			notchCalibrate(_cleanedPointsX, _cleanedPointsY, _notchPointsX, _notchPointsY, _noOfNotches, _aAffineCoeffs, _aBoundaryAngles);
			//apply the calibration to the original measured values including any tertiaries; we don't care about the angles
			cleanCalPoints(_tempCalPointsX, _tempCalPointsY, _notchAngleDefaults, _cleanedPointsX, _cleanedPointsY, _notchPointsX, _notchPointsY, _aNotchStatus);
			float transformedX[_noOfNotches+1];
			float transformedY[_noOfNotches+1];
			transformCalPoints(_cleanedPointsX, _cleanedPointsY, transformedX, transformedY, _aFitCoeffsX, _aFitCoeffsY, _aAffineCoeffs, _aBoundaryAngles);
			//compute the angles for those notches into _measuredNotchAngles, using the default angles for the diagonals
			computeStickAngles(transformedX, transformedY, _measuredNotchAngles);
			//clean full cal points again, feeding those angles in
			cleanCalPoints(_tempCalPointsX, _tempCalPointsY, _measuredNotchAngles, _cleanedPointsX, _cleanedPointsY, _notchPointsX, _notchPointsY, _aNotchStatus);
			//clear unused notch angles
			cleanNotches(_aNotchAngles, _measuredNotchAngles, _aNotchStatus);
			//clean full cal points again again, feeding those measured angles in for missing tertiary notches
			cleanCalPoints(_tempCalPointsX, _tempCalPointsY, _aNotchAngles, _cleanedPointsX, _cleanedPointsY, _notchPointsX, _notchPointsY, _aNotchStatus);
			//linearize again
			linearizeCal(_cleanedPointsX, _cleanedPointsY, _cleanedPointsX, _cleanedPointsY, _aFitCoeffsX, _aFitCoeffsY);
			//notchCalibrate again
			notchCalibrate(_cleanedPointsX, _cleanedPointsY, _notchPointsX, _notchPointsY, _noOfNotches, _aAffineCoeffs, _aBoundaryAngles);
		}
	}
	//Undo Calibration using Z-button
	if(hardware.Z && _undoCal && !_undoCalPressed) {
		_undoCalPressed = true;
		if(_currentCalStep % 2 == 0 && _currentCalStep < 32 && _currentCalStep != 0 ) {
			//If it's measuring zero, go back to the previous zero
			_currentCalStep --;
			_currentCalStep --;
		} else if(_currentCalStep % 2 == 1 && _currentCalStep < 32 && _currentCalStep != 0 ) {
			//If it's measuring a notch, go back to the zero before the previous notch
			_currentCalStep -= 3;
			_currentCalStep = max(_currentCalStep, 0);
		} else if(_currentCalStep > 32) {
			//We can go directly between notches when adjusting notches
			_currentCalStep --;
		}
		if(!_calAStick){
			int notchIndex = _notchAdjOrder[min(_currentCalStep-_noOfCalibrationPoints, _noOfAdjNotches-1)];//limit this so it doesn't access outside the array bounds
			while((_currentCalStep >= _noOfCalibrationPoints) && (_cNotchStatus[notchIndex] == _tertiaryNotchInactive) && (_currentCalStep < _noOfCalibrationPoints + _noOfAdjNotches)){//this non-diagonal notch was not calibrated
				//skip to the next valid notch
				_currentCalStep--;
				notchIndex = _notchAdjOrder[min(_currentCalStep-_noOfCalibrationPoints, _noOfAdjNotches-1)];//limit this so it doesn't access outside the array bounds
			}
		} else if(_calAStick){
			int notchIndex = _notchAdjOrder[min(_currentCalStep-_noOfCalibrationPoints, _noOfAdjNotches-1)];//limit this so it doesn't access outside the array bounds
			while((_currentCalStep >= _noOfCalibrationPoints) && (_aNotchStatus[notchIndex] == _tertiaryNotchInactive) && (_currentCalStep < _noOfCalibrationPoints + _noOfAdjNotches)){//this non-diagonal notch was not calibrated
				//skip to the next valid notch
				_currentCalStep--;
				notchIndex = _notchAdjOrder[min(_currentCalStep-_noOfCalibrationPoints, _noOfAdjNotches-1)];//limit this so it doesn't access outside the array bounds
			}
		}
	} else if(!hardware.Z) {
		_undoCalPressed = false;
	}

	//Advance Calibration Using L or R triggers
	static float advanceCalAccumulator = 0.0;
	if((hardware.L || hardware.R) && _advanceCal){// && !_advanceCalPressed){
		advanceCalAccumulator = 0.96*advanceCalAccumulator + 0.04;
	} else {
		advanceCalAccumulator = 0.96*advanceCalAccumulator;
	}
	if(advanceCalAccumulator > 0.75 && !_advanceCalPressed){
		_advanceCalPressed = true;
		if (!_calAStick){
			if(_currentCalStep < _noOfCalibrationPoints){//still collecting points
				collectCalPoints(_calAStick, _currentCalStep,_tempCalPointsX,_tempCalPointsY, _pinList);
			}
			_currentCalStep ++;
			if(_currentCalStep >= 2 && _currentCalStep != _noOfNotches*2) {//don't undo at the beginning of collection or notch adjust
				_undoCal = true;
			} else {
				_undoCal = false;
			}
			if(_currentCalStep == _noOfCalibrationPoints){//done collecting points
				Serial.println("finished collecting the calibration points for the C stick");
				//make temp temp cal points that are missing all tertiary notches so that we get a neutral grid
				float tempCalPointsX[_noOfCalibrationPoints];
				float tempCalPointsY[_noOfCalibrationPoints];
				stripCalPoints(_tempCalPointsX, _tempCalPointsY, tempCalPointsX, tempCalPointsY);
				//clean the stripped calibration points, use default angles
				cleanCalPoints(tempCalPointsX, tempCalPointsY, _notchAngleDefaults, _cleanedPointsX, _cleanedPointsY, _notchPointsX, _notchPointsY, _cNotchStatus);
				linearizeCal(_cleanedPointsX, _cleanedPointsY, _cleanedPointsX, _cleanedPointsY, _cFitCoeffsX, _cFitCoeffsY);
				notchCalibrate(_cleanedPointsX, _cleanedPointsY, _notchPointsX, _notchPointsY, _noOfNotches, _cAffineCoeffs, _cBoundaryAngles);
				//apply the calibration to the original measured values including any tertiaries; we don't care about the angles
				cleanCalPoints(_tempCalPointsX, _tempCalPointsY, _notchAngleDefaults, _cleanedPointsX, _cleanedPointsY, _notchPointsX, _notchPointsY, _cNotchStatus);
				float transformedX[_noOfNotches+1];
				float transformedY[_noOfNotches+1];
				transformCalPoints(_cleanedPointsX, _cleanedPointsY, transformedX, transformedY, _cFitCoeffsX, _cFitCoeffsY, _cAffineCoeffs, _cBoundaryAngles);
				//compute the angles for those notches into _measuredNotchAngles, using the default angles for the diagonals
				computeStickAngles(transformedX, transformedY, _measuredNotchAngles);
				//clean full cal points again, feeding those angles in
				cleanCalPoints(_tempCalPointsX, _tempCalPointsY, _measuredNotchAngles, _cleanedPointsX, _cleanedPointsY, _notchPointsX, _notchPointsY, _cNotchStatus);
				//clear unused notch angles
				cleanNotches(_cNotchAngles, _measuredNotchAngles, _cNotchStatus);
				//clean full cal points again again, feeding those measured angles in for missing tertiary notches
				cleanCalPoints(_tempCalPointsX, _tempCalPointsY, _cNotchAngles, _cleanedPointsX, _cleanedPointsY, _notchPointsX, _notchPointsY, _cNotchStatus);
				//linearize again
				linearizeCal(_cleanedPointsX, _cleanedPointsY, _cleanedPointsX, _cleanedPointsY, _cFitCoeffsX, _cFitCoeffsY);
				//notchCalibrate again
				notchCalibrate(_cleanedPointsX, _cleanedPointsY, _notchPointsX, _notchPointsY, _noOfNotches, _cAffineCoeffs, _cBoundaryAngles);
			}
			int notchIndex = _notchAdjOrder[min(_currentCalStep-_noOfCalibrationPoints, _noOfAdjNotches-1)];//limit this so it doesn't access outside the array bounds
			while((_currentCalStep >= _noOfCalibrationPoints) && (_cNotchStatus[notchIndex] == _tertiaryNotchInactive) && (_currentCalStep < _noOfCalibrationPoints + _noOfAdjNotches)){//this non-diagonal notch was not calibrated
				//skip to the next valid notch
				_currentCalStep++;
				notchIndex = _notchAdjOrder[min(_currentCalStep-_noOfCalibrationPoints, _noOfAdjNotches-1)];//limit this so it doesn't access outside the array bounds
			}
			if(_currentCalStep >= _noOfCalibrationPoints + _noOfAdjNotches){//done adjusting notches
				Serial.println("finished adjusting notches for the C stick");
				EEPROM.put(_eepromCPointsX, _tempCalPointsX);
				EEPROM.put(_eepromCPointsY, _tempCalPointsY);
				EEPROM.put(_eepromCNotchAngles, _cNotchAngles);
				controls.autoInit = 0;
				EEPROM.put(_eepromAutoInit, controls.autoInit);
				Serial.println("calibration points stored in EEPROM");
				cleanCalPoints(_tempCalPointsX, _tempCalPointsY, _cNotchAngles, _cleanedPointsX, _cleanedPointsY, _notchPointsX, _notchPointsY, _cNotchStatus);
				Serial.println("calibration points cleaned");
				linearizeCal(_cleanedPointsX, _cleanedPointsY, _cleanedPointsX, _cleanedPointsY, _cFitCoeffsX, _cFitCoeffsY);
				Serial.println("C stick linearized");
				notchCalibrate(_cleanedPointsX, _cleanedPointsY, _notchPointsX, _notchPointsY, _noOfNotches, _cAffineCoeffs, _cBoundaryAngles);
				_currentCalStep = -1;
				_advanceCal = false;
			}
		}
		else if (_calAStick){
			Serial.println("Current step:");
			Serial.println(_currentCalStep);
			if(_currentCalStep < _noOfCalibrationPoints){//still collecting points
				collectCalPoints(_calAStick, _currentCalStep,_tempCalPointsX,_tempCalPointsY, _pinList);
			}
			_currentCalStep ++;
			if(_currentCalStep >= 2 && _currentCalStep != _noOfCalibrationPoints) {//don't undo at the beginning of collection or notch adjust
				_undoCal = true;
			} else {
				_undoCal = false;
			}
			if(_currentCalStep == _noOfCalibrationPoints){//done collecting points
				//make temp temp cal points that are missing all tertiary notches so that we get a neutral grid
				float tempCalPointsX[_noOfCalibrationPoints];
				float tempCalPointsY[_noOfCalibrationPoints];
				stripCalPoints(_tempCalPointsX, _tempCalPointsY, tempCalPointsX, tempCalPointsY);
				//clean the stripped calibration points, use default angles
				cleanCalPoints(tempCalPointsX, tempCalPointsY, _notchAngleDefaults, _cleanedPointsX, _cleanedPointsY, _notchPointsX, _notchPointsY, _aNotchStatus);
				linearizeCal(_cleanedPointsX, _cleanedPointsY, _cleanedPointsX, _cleanedPointsY, _aFitCoeffsX, _aFitCoeffsY);
				notchCalibrate(_cleanedPointsX, _cleanedPointsY, _notchPointsX, _notchPointsY, _noOfNotches, _aAffineCoeffs, _aBoundaryAngles);
				//apply the calibration to the original measured values including any tertiaries; we don't care about the angles
				cleanCalPoints(_tempCalPointsX, _tempCalPointsY, _notchAngleDefaults, _cleanedPointsX, _cleanedPointsY, _notchPointsX, _notchPointsY, _aNotchStatus);
				float transformedX[_noOfNotches+1];
				float transformedY[_noOfNotches+1];
				transformCalPoints(_cleanedPointsX, _cleanedPointsY, transformedX, transformedY, _aFitCoeffsX, _aFitCoeffsY, _aAffineCoeffs, _aBoundaryAngles);
				//compute the angles for those notches into _measuredNotchAngles, using the default angles for the diagonals
				computeStickAngles(transformedX, transformedY, _measuredNotchAngles);
				//clean full cal points again, feeding those angles in
				cleanCalPoints(_tempCalPointsX, _tempCalPointsY, _measuredNotchAngles, _cleanedPointsX, _cleanedPointsY, _notchPointsX, _notchPointsY, _aNotchStatus);
				//clear unused notch angles
				cleanNotches(_aNotchAngles, _measuredNotchAngles, _aNotchStatus);
				//clean full cal points again again, feeding those measured angles in for missing tertiary notches
				cleanCalPoints(_tempCalPointsX, _tempCalPointsY, _aNotchAngles, _cleanedPointsX, _cleanedPointsY, _notchPointsX, _notchPointsY, _aNotchStatus);
				//linearize again
				linearizeCal(_cleanedPointsX, _cleanedPointsY, _cleanedPointsX, _cleanedPointsY, _aFitCoeffsX, _aFitCoeffsY);
				//notchCalibrate again
				notchCalibrate(_cleanedPointsX, _cleanedPointsY, _notchPointsX, _notchPointsY, _noOfNotches, _aAffineCoeffs, _aBoundaryAngles);
			}
			int notchIndex = _notchAdjOrder[min(_currentCalStep-_noOfCalibrationPoints, _noOfAdjNotches-1)];//limit this so it doesn't access outside the array bounds
			while((_currentCalStep >= _noOfCalibrationPoints) && (_aNotchStatus[notchIndex] == _tertiaryNotchInactive) && (_currentCalStep < _noOfCalibrationPoints + _noOfAdjNotches)){//this non-diagonal notch was not calibrated
				//skip to the next valid notch
				_currentCalStep++;
				notchIndex = _notchAdjOrder[min(_currentCalStep-_noOfCalibrationPoints, _noOfAdjNotches-1)];//limit this so it doesn't access outside the array bounds
			}
			if(_currentCalStep >= _noOfCalibrationPoints + _noOfAdjNotches){//done adjusting notches
				Serial.println("finished adjusting notches for the A stick");
				EEPROM.put(_eepromAPointsX, _tempCalPointsX);
				EEPROM.put(_eepromAPointsY, _tempCalPointsY);
				EEPROM.put(_eepromANotchAngles, _aNotchAngles);
				controls.autoInit = 0;
				EEPROM.put(_eepromAutoInit, controls.autoInit);
				Serial.println("calibration points stored in EEPROM");
				cleanCalPoints(_tempCalPointsX, _tempCalPointsY, _aNotchAngles, _cleanedPointsX, _cleanedPointsY, _notchPointsX, _notchPointsY, _aNotchStatus);
				Serial.println("calibration points cleaned");
				linearizeCal(_cleanedPointsX, _cleanedPointsY, _cleanedPointsX, _cleanedPointsY, _aFitCoeffsX, _aFitCoeffsY);
				Serial.println("A stick linearized");
				notchCalibrate(_cleanedPointsX, _cleanedPointsY, _notchPointsX, _notchPointsY, _noOfNotches, _aAffineCoeffs, _aBoundaryAngles);
				_currentCalStep = -1;
				_advanceCal = false;
			}
		}
	} else if(advanceCalAccumulator <= 0.25) {
		_advanceCalPressed = false;
	}
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

void showRumble(const int time, Buttons &btn, HardwareButtons &hardware, ControlConfig &controls) {
	btn.Cx = (uint8_t) 127;
	btn.Cy = (uint8_t) (controls.rumble + 127.5);
	clearButtons(time, btn, hardware);

	EEPROM.put(_eepromRumble, controls.rumble);
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
	btn.Ax = (uint8_t) (controls.autoInit*100 - 50 + 127.5);
	btn.Ay = (uint8_t) (controls.autoInit*100 - 50 + 127.5);
	btn.Cx = (uint8_t) (controls.autoInit*100 - 50 + 127.5);
	btn.Cy = (uint8_t) (controls.autoInit*100 - 50 + 127.5);

	clearButtons(2000, btn, hardware);

	EEPROM.put(_eepromAutoInit, controls.autoInit);
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

	btn.Cx = (uint8_t) (controls.xSnapback + 127.5);
	btn.Cy = (uint8_t) (controls.ySnapback + 127.5);

	clearButtons(2000, btn, hardware);

	EEPROM.put(_eepromxSnapback,controls.xSnapback);
	EEPROM.put(_eepromySnapback,controls.ySnapback);
}
void adjustSmoothing(const WhichAxis axis, const Increase increase, Buttons &btn, HardwareButtons &hardware, ControlConfig &controls, FilterGains &gains, FilterGains &normGains) {
	Serial.println("Adjusting Smoothing");
	if (axis == XAXIS && increase == INCREASE) {
		gains.xSmoothing = gains.xSmoothing + 0.1;
		if(gains.xSmoothing > controls.smoothingMax) {
			gains.xSmoothing = controls.smoothingMax;
		}
		EEPROM.put(_eepromxSmoothing, gains.xSmoothing);
		Serial.print("X Smoothing increased to:");
		Serial.println(gains.xSmoothing);
	} else if(axis == XAXIS && increase == DECREASE) {
		gains.xSmoothing = gains.xSmoothing - 0.1;
		if(gains.xSmoothing < controls.smoothingMin) {
			gains.xSmoothing = controls.smoothingMin;
		}
		EEPROM.put(_eepromxSmoothing, gains.xSmoothing);
		Serial.print("X Smoothing decreased to:");
		Serial.println(gains.xSmoothing);
	} else if(axis == YAXIS && increase == INCREASE) {
		gains.ySmoothing = gains.ySmoothing + 0.1;
		if (gains.ySmoothing > controls.smoothingMax) {
			gains.ySmoothing = controls.smoothingMax;
		}
		EEPROM.put(_eepromySmoothing, gains.ySmoothing);
		Serial.print("Y Smoothing increased to:");
		Serial.println(gains.ySmoothing);
	} else if(axis == YAXIS && increase == DECREASE) {
		gains.ySmoothing = gains.ySmoothing - 0.1;
		if (gains.ySmoothing < controls.smoothingMin) {
			gains.ySmoothing = controls.smoothingMin;
		}
		EEPROM.put(_eepromySmoothing, gains.ySmoothing);
		Serial.print("Y Smoothing decreased to:");
		Serial.println(gains.ySmoothing);
	}

	//recompute the intermediate gains used directly by the kalman filter
	recomputeGains(gains, normGains);

	btn.Cx = (uint8_t) (127.5 + (gains.xSmoothing * 10));
	btn.Cy = (uint8_t) (127.5 + (gains.ySmoothing * 10));

	clearButtons(2000, btn, hardware);
}
void showAstickSettings(Buttons &btn, HardwareButtons &hardware, const ControlConfig &controls, FilterGains &gains) {
	//Snapback on A-stick
	btn.Ax = (uint8_t) (controls.xSnapback + 127.5);
	btn.Ay = (uint8_t) (controls.ySnapback + 127.5);

	//Smoothing on C-stick
	btn.Cx = (uint8_t) (127.5 + (gains.xSmoothing * 10));
	btn.Cy = (uint8_t) (127.5 + (gains.ySmoothing * 10));

	clearButtons(2000, btn, hardware);
}
void adjustCstickSmoothing(const WhichAxis axis, const Increase increase, Buttons &btn, HardwareButtons &hardware, ControlConfig &controls, FilterGains &gains, FilterGains &normGains) {
	Serial.println("Adjusting C-Stick Smoothing");
	if (axis == XAXIS && increase == INCREASE) {
		gains.cXSmoothing = gains.cXSmoothing + 0.1;
		if(gains.cXSmoothing > controls.smoothingMax) {
			gains.cXSmoothing = controls.smoothingMax;
		}
		EEPROM.put(_eepromCxSmoothing, gains.cXSmoothing);
		Serial.print("C-Stick X Smoothing increased to:");
		Serial.println(gains.cXSmoothing);
	} else if(axis == XAXIS && increase == DECREASE) {
		gains.cXSmoothing = gains.cXSmoothing - 0.1;
		if(gains.cXSmoothing < controls.smoothingMin) {
			gains.cXSmoothing = controls.smoothingMin;
		}
		EEPROM.put(_eepromCxSmoothing, gains.cXSmoothing);
		Serial.print("C-Stick X Smoothing decreased to:");
		Serial.println(gains.cXSmoothing);
	} else if(axis == YAXIS && increase == INCREASE) {
		gains.cYSmoothing = gains.cYSmoothing + 0.1;
		if (gains.cYSmoothing > controls.smoothingMax) {
			gains.cYSmoothing = controls.smoothingMax;
		}
		EEPROM.put(_eepromCySmoothing, gains.cYSmoothing);
		Serial.print("C-Stick Y Smoothing increased to:");
		Serial.println(gains.cYSmoothing);
	} else if(axis == YAXIS && increase == DECREASE) {
		gains.cYSmoothing = gains.cYSmoothing - 0.1;
		if (gains.cYSmoothing < controls.smoothingMin) {
			gains.cYSmoothing = controls.smoothingMin;
		}
		EEPROM.put(_eepromCySmoothing, gains.cYSmoothing);
		Serial.print("C-Stick Y Smoothing decreased to:");
		Serial.println(gains.cYSmoothing);
	}

	//recompute the intermediate gains used directly by the kalman filter
	recomputeGains(gains, normGains);

	btn.Cx = (uint8_t) (127.5 + (gains.cXSmoothing * 10));
	btn.Cy = (uint8_t) (127.5 + (gains.cYSmoothing * 10));

	clearButtons(2000, btn, hardware);
}
void adjustCstickOffset(const WhichAxis axis, const Increase increase, Buttons &btn, HardwareButtons &hardware, ControlConfig &controls) {
	Serial.println("Adjusting C-stick Offset");
	if(axis == XAXIS && increase == INCREASE) {
		controls.cXOffset++;
		if(controls.cXOffset > controls.cMax) {
			controls.cXOffset = controls.cMax;
		}
		//EEPROM.put(_eepromcXOffset, controls.cXOffset);
		setCXOffsetSetting(controls.cXOffset);
		Serial.print("X offset increased to:");
		Serial.println(controls.cXOffset);
	} else if(axis == XAXIS && increase == DECREASE) {
		controls.cXOffset--;
		if(controls.cXOffset < controls.cMin) {
			controls.cXOffset = controls.cMin;
		}
		//EEPROM.put(_eepromcXOffset, controls.cXOffset);
		setCXOffsetSetting(controls.cXOffset);
		Serial.print("X offset decreased to:");
		Serial.println(controls.cXOffset);
	} else if(axis == YAXIS && increase == INCREASE) {
		controls.cYOffset++;
		if(controls.cYOffset > controls.cMax) {
			controls.cYOffset = controls.cMax;
		}
		//EEPROM.put(_eepromcYOffset, controls.cYOffset);
		setCYOffsetSetting(controls.cYOffset);
		Serial.print("Y offset increased to:");
		Serial.println(controls.cYOffset);
	} else if(axis == YAXIS && increase == DECREASE) {
		controls.cYOffset--;
		if(controls.cYOffset < controls.cMin) {
			controls.cYOffset = controls.cMin;
		}
		//EEPROM.put(_eepromcYOffset, controls.cYOffset);
		setCYOffsetSetting(controls.cYOffset);
		Serial.print("Y offset decreased to:");
		Serial.println(controls.cYOffset);
	}

	btn.Cx = (uint8_t) (127.5 + controls.cXOffset);
	btn.Cy = (uint8_t) (127.5 + controls.cYOffset);

	clearButtons(2000, btn, hardware);
}
void showCstickSettings(Buttons &btn, HardwareButtons &hardware, ControlConfig &controls, FilterGains &gains) {
	//Snapback/smoothing on A-stick
	btn.Ax = (uint8_t) (127.5 + (gains.cXSmoothing * 10));
	btn.Ay = (uint8_t) (127.5 + (gains.cYSmoothing * 10));

	//Smoothing on C-stick
	btn.Cx = (uint8_t) (127.5 + controls.cXOffset);
	btn.Cy = (uint8_t) (127.5 + controls.cYOffset);

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

	//EEPROM.put(_eepromLOffset, controls.lTriggerOffset);
	setLOffsetSetting(controls.lTriggerOffset);
	//EEPROM.put(_eepromROffset, controls.rTriggerOffset);
	setROffsetSetting(controls.rTriggerOffset);

	if(controls.lTriggerOffset > 99) {
		btn.Ax = (uint8_t) (127.5 + 100);
		btn.Cx = (uint8_t) (127.5 + controls.lTriggerOffset-100);
	} else {
		btn.Cx = (uint8_t) (127.5 + controls.lTriggerOffset);
	}
	if(controls.rTriggerOffset > 99) {
		btn.Ay = (uint8_t) (127.5 + 100);
		btn.Cy = (uint8_t) (127.5 + controls.rTriggerOffset-100);
	} else {
		btn.Cy = (uint8_t) (127.5 + controls.rTriggerOffset);
	}

	clearButtons(250, btn, hardware);
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
	//EEPROM.put(_eepromJump,controls.jumpConfig);
	setJumpSetting(controls.jumpConfig);
	setJump(controls);
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
	//EEPROM.put(_eepromLToggle, controls.lConfig);
	setLSetting(controls.lConfig);
	//EEPROM.put(_eepromRToggle, controls.rConfig);
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
	btn.Ay = (uint8_t) (127.5 + triggerConflict);
	btn.Ax = (uint8_t) (127.5 + controls.lConfig + 1);
	btn.Cy = (uint8_t) (127.5 + triggerConflict);
	btn.Cx = (uint8_t) (127.5 + controls.rConfig + 1);

	clearButtons(2000, btn, hardware);
}
void initializeButtons(Buttons &btn,int &startUpLa, int &startUpRa){
	//set the analog stick values to the chosen center value that will be reported to the console on startup
	//We choose 127 for this, and elsewhere we use an offset of 127.5 truncated to int in order to round properly
	btn.Ax = 127;
	btn.Ay = 127;
	btn.Cx = 127;
	btn.Cy = 127;

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
