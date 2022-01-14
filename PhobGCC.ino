//This software uses bits of code from GoodDoge's Dogebawx project, which was the initial starting point: https://github.com/DogeSSBM/DogeBawx

#include <math.h>
#include <curveFitting.h>
#include <EEPROM.h>
#include <eigen.h>
#include <Eigen/LU>
#include <ADC.h>
#include <Bounce2.h>

using namespace Eigen;

#define CMD_LENGTH_SHORT 5
#define CMD_LENGTH_LONG 13
#define ORIGIN_LENGTH 41
#define POLL_LENGTH 33
#define CALIBRATION_POINTS 17

#define GC_FREQUENCY 1250000
#define PC_FREQUENCY 1000000
#define PULSE_FREQ_CUTOFF 291666

/////defining which pin is what on the teensy
const int _pinLa = 16;
const int _pinRa = 23;
const int _pinL = 13;
const int _pinR = 3;
const int _pinAx = 15;
const int _pinAy = 14;
//const int _pinCx = 21;
//const int _pinCy = 22;
const int _pinCx = 22;
const int _pinCy = 21;
const int _pinRX = 9;
const int _pinTX = 10;
const int _pinDr = 7;
const int _pinDu = 18;
const int _pinDl = 17;
const int _pinDd = 8;
const int _pinX = 1;
const int _pinY = 2;
const int _pinA = 4;
const int _pinB = 6;
const int _pinZ = 0;
const int _pinS = 19;
int _pinZSwappable = _pinZ;
int _pinXSwappable = _pinX;
int _pinYSwappable = _pinY;
int _jumpConfig = 0;

///// Values used for dealing with snapback in the Kalman Filter, a 6th power relationship between distance to center and ADC/acceleration variance is used, this was arrived at by trial and error
const float _accelVarFast = 0.5; //governs the acceleration variation around the edge of the gate, higher value means less filtering
const float _accelVarSlow = 0.000001; //governs the acceleration variation with the stick centered, higher value means less filtering
const float _ADCVarFast = 0.1; //governs the ADC variation around the edge of the gate, higher vaule means more filtering
const float _ADCVarMax = 10; //maximum allowable value for the ADC variation
const float _ADCVarMin = 0.1; //minimum allowable value for the ADC variation
const float _x1 = 100; //maximum stick distance from center
const float _x6 = _x1*_x1*_x1*_x1*_x1*_x1; //maximum distance to the 6th power
const float _aAccelVar = (_accelVarFast - _accelVarSlow)/_x6; //first coefficient used to calculate the actual acceleration variation
const float _bAccelVar = _accelVarSlow; //Second coefficient used to calculate the actual acceleration variation
const float _damping = 1; //amount of damping used in the stick model
//ADC variance is used to compensate for differing amounts of snapback, so it is not a constant, can be set by user
float _ADCVarSlow = 0.2; //default value
float _aADCVar = (_ADCVarFast - _ADCVarSlow)/_x6; //first coefficient used to calculate the actual ADC variation
float _bADCVar = _ADCVarSlow; //second coefficient used to calculate the actual ADC variation

//////values used to determine how much large of a region will count as being "in a notch"

const float _marginAngle = 5.0/100.0; //angle range(+/-) in radians that will be collapsed down to the ideal angle
const float _tightAngle = 0.1/100.0;//angle range(+/-) in radians that the margin region will be collapsed down to, found that having a small value worked better for the transform than 0

//////values used for calibration
const int _notCalibrating = -1;
bool	_calAStick = true; //determines which stick is being calibrated (if false then calibrate the c-stick)
int _currentCalStep; //keeps track of which caliblration step is active, -1 means calibration is not running
bool _notched = false; //keeps track of whether or not the controller has firefox notches
const int _calibrationPoints = 9; //number of calibration points for the c-stick and a-stick for a controller without notches
const int _calibrationPointsNotched = 17; //number of calibration points for thea-stick for a controller with notches
float _cleanedPointsX[17]; //array to hold the x coordinates of the calibration points
float _cleanedPointsY[17]; //array to hold the y coordinates of the calibration points
const int _fitOrder = 3; //fit order used in the linearization step
float _aFitCoeffsX[_fitOrder+1]; //coefficients for linearizing the X axis of the a-stick
float _aFitCoeffsY[_fitOrder+1]; //coefficients for linearizing the Y axis of the a-stick
float _cFitCoeffsX[_fitOrder+1]; //coefficients for linearizing the Y axis of the c-stick
float _cFitCoeffsY[_fitOrder+1]; //coefficients for linearizing the Y axis of the c-stick
float _aAffineCoeffs[_calibrationPointsNotched-1][6]; //affine transformation coefficients for all regions of the a-stick
float _cAffineCoeffs[_calibrationPointsNotched-1][6]; //affine transformation coefficients for all regions of the c-stick
float _aBoundaryAngles[_calibrationPointsNotched-1]; //angles at the boundaries between regions of the a-stick
float _cBoundaryAngles[_calibrationPointsNotched-1]; //angles at the boundaries between regions of the c-stick
float _tempCalPointsX[(_calibrationPointsNotched-1)*2]; //temporary storage for the x coordinate points collected during calibration before the are cleaned and put into _cleanedPointsX
float _tempCalPointsY[(_calibrationPointsNotched-1)*2]; //temporary storage for the y coordinate points collected during calibration before the are cleaned and put into _cleanedPointsY

//index values to store data into eeprom
const int _bytesPerFloat = 4;
const int _eepromAPointsX = 0;
const int _eepromAPointsY = _eepromAPointsX+_calibrationPointsNotched*_bytesPerFloat;
const int _eepromCPointsX = _eepromAPointsY+_calibrationPointsNotched*_bytesPerFloat;
const int _eepromCPointsY = _eepromCPointsX+_calibrationPointsNotched*_bytesPerFloat;
const int _eepromNotched = _eepromCPointsY+_calibrationPoints*_bytesPerFloat;
const int _eepromADCVar = _eepromNotched+_bytesPerFloat;
const int _eepromJump = _eepromADCVar+_bytesPerFloat;


Bounce bounceDr = Bounce();
Bounce bounceDu = Bounce(); 
Bounce bounceDl = Bounce(); 
Bounce bounceDd = Bounce(); 

ADC *adc = new ADC();

union Buttons{
	uint8_t arr[10];
	struct {
		
				// byte 0
		uint8_t A : 1;
		uint8_t B : 1;
		uint8_t X : 1;
		uint8_t Y : 1;
		uint8_t S : 1;
		uint8_t orig : 1;
		uint8_t errL : 1;
		uint8_t errS : 1;
		
		// byte 1
		uint8_t Dl : 1;
		uint8_t Dr : 1;
		uint8_t Dd : 1;
		uint8_t Du : 1;
		uint8_t Z : 1;
		uint8_t R : 1;
		uint8_t L : 1;
		uint8_t high : 1;

		//byte 2-7
		uint8_t Ax : 8;
		uint8_t Ay : 8;
		uint8_t Cx : 8;
		uint8_t Cy : 8;
		uint8_t La : 8;
		uint8_t Ra : 8;

		// magic byte 8 & 9 (only used in origin cmd)
		// have something to do with rumble motor status???
		// ignore these, they are magic numbers needed
		// to make a cmd response work
		uint8_t magic1 : 8;
		uint8_t magic2 : 8;
	};
}btn;

float _aStickX;
float _aStickY;
float _cStickX;
float _cStickY;

volatile int _writeQueue = 0;



unsigned int _lastMicros;
bool _running = false;

VectorXf _xState(2);
VectorXf _yState(2);
MatrixXf _xP(2,2);
MatrixXf _yP(2,2);


static uint8_t probeResponse[CMD_LENGTH_LONG] = {
	0x08,0x08,0x0F,0xE8,
	0x08,0x08,0x08,0x08,
	0x08,0x08,0x08,0xEF,
	0xFF};
volatile uint8_t pollResponse[POLL_LENGTH] = {
0x08,0x08,0x08,0x08,
0x0F,0x08,0x08,0x08,
0xE8,0xEF,0xEF,0xEF,
0xE8,0xEF,0xEF,0xEF,
0xE8,0xEF,0xEF,0xEF,
0xE8,0xEF,0xEF,0xEF,
0x08,0xEF,0xEF,0x08,
0x08,0xEF,0xEF,0x08,
0xFF};
static uint8_t originResponse[ORIGIN_LENGTH] = {
	0x08,0x08,0x08,0x08,
	0x0F,0x08,0x08,0x08,
	0xE8,0xEF,0xEF,0xEF,
	0xE8,0xEF,0xEF,0xEF,
	0xE8,0xEF,0xEF,0xEF,
	0xE8,0xEF,0xEF,0xEF,
	0x08,0xEF,0xEF,0x08,
	0x08,0xEF,0xEF,0x08,
	0x08,0x08,0x08,0x08,
	0x08,0x08,0x08,0x08,
	0xFF};

int cmd[CMD_LENGTH_LONG];
uint8_t cmdByte;

void setup() {
	
	//start USB serial
	Serial.begin(57600);
	Serial.println("test");
	delay(1000);
	
	//get the calibration points from EEPROM memory and find all the coefficients
	//Analog Stick
	//EEPROM.get(_eepromNotched, _notched);
	EEPROM.get(_eepromJump, _jumpConfig);
	setJump(_jumpConfig);
	EEPROM.get(_eepromADCVar, _ADCVarSlow);
	Serial.print("the _ADCVarSlow value from eeprom is:");
	Serial.println(_ADCVarSlow);
	if(std::isnan(_ADCVarSlow)){
		_ADCVarSlow = _ADCVarMin;
		Serial.print("the _ADCVarSlow value was adjusted to:");
		Serial.println(_ADCVarSlow);
	}
	if(_ADCVarSlow >_ADCVarMax){
		_ADCVarSlow = _ADCVarMax;
	}
	else if(_ADCVarSlow < _ADCVarMin){
		_ADCVarSlow = _ADCVarMin;
	}
	setADCVar(&_aADCVar, &_bADCVar, _ADCVarSlow);
	
	EEPROM.get(_eepromAPointsX, _cleanedPointsX);
	EEPROM.get(_eepromAPointsY, _cleanedPointsY);
	stickCal(_cleanedPointsX,_cleanedPointsY,_notched,_aFitCoeffsX,_aFitCoeffsY,_aAffineCoeffs,_aBoundaryAngles);
	
	//now for the C stick
	EEPROM.get(_eepromCPointsX, _cleanedPointsX);
	EEPROM.get(_eepromCPointsY, _cleanedPointsY);
	stickCal(_cleanedPointsX,_cleanedPointsY,false,_cFitCoeffsX,_cFitCoeffsY,_cAffineCoeffs,_cBoundaryAngles);
	
	//set some of the unused values in the message response
	btn.errS = 0;
	btn.errL = 0;
	btn.orig = 0;
	btn.high = 1;
	
	//
	_currentCalStep = _notCalibrating;

	
	_xState << 0,0;
	_yState << 0,0;
	_xP << 1000,0,0,1000;
	_yP << 1000,0,0,1000;
	_lastMicros = micros();
	
	adc->adc0->setAveraging(8); // set number of averages
  adc->adc0->setResolution(12); // set bits of resolution
  adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::HIGH_SPEED ); // change the conversion speed
  adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_LOW_SPEED ); // change the sampling speed
	
	setPinModes();
	
	int serialFreq = findFreq();
	Serial.print("starting hw serial at freq:");
	Serial.println(serialFreq);
	//start hardware serail
	Serial2.begin(serialFreq);
	//attach the interrupt which will call the communicate function when the data line transitions from high to low
	attachInterrupt(_pinRX, communicate, FALLING);
}

void loop() {
	//read the controllers buttons
	readButtons();
	//read the analog inputs
	readSticks();
	//check to see if we are calibrating
	if(_currentCalStep >= 0){
			calibrate(_calAStick);
	}
	//check if we should be reporting values yet
	if(btn.B && !_running){
		Serial.println("Starting to report values");
		_running=true;
	}
	//update the pole message so new data will be sent to the gamecube
	if(_running){
		setPole();
	}

}
int findFreq() {
	//try to determine the speed the hardware serial needs to run at by counting the probe command pulse widths
	int serialFreq = PC_FREQUENCY;
	noInterrupts();
	pinMode(_pinRX,INPUT);
	unsigned int counter = 0;
	unsigned int start = 0;
	unsigned int duration = 0;
	ARM_DEMCR |= ARM_DEMCR_TRCENA;
	ARM_DWT_CTRL |= ARM_DWT_CTRL_CYCCNTENA;
	//attempt to count a bunch of 0 bits (may also catch some stop bits which are short, will skew the results)
	for(int i = 0; i<64;i++){
		//wait for the pause between pulses (
		start = ARM_DWT_CYCCNT;
		duration = 0;
		while(duration < (F_CPU*0.00001)){
			if(digitalReadFast(_pinRX)){
				duration = ARM_DWT_CYCCNT-start;
			}
			else {
				start = ARM_DWT_CYCCNT;
				duration = 0;
			}
		}
		
		//measure the clock cycles of the first 0 bit
		while(digitalReadFast(_pinRX)){}
		start = ARM_DWT_CYCCNT;
		while(!digitalReadFast(_pinRX)){}
		counter += ARM_DWT_CYCCNT-start;
	}
	counter = counter>>6;

	int pulseWidthFreq = F_CPU/counter;
	Serial.print("measured pulse width freq:");
	Serial.println(pulseWidthFreq);
	//pulse widths on my usb adapter are ~4us, gamecube/wii's are supposed to be 3us, this should check directly between them
	if(pulseWidthFreq < PULSE_FREQ_CUTOFF){
		serialFreq = PC_FREQUENCY;
	}
	else{
		serialFreq = GC_FREQUENCY;
	}
	interrupts();
	return serialFreq;
}
void setPinModes(){
	pinMode(0,INPUT_PULLUP);
	pinMode(1,INPUT_PULLUP);
	pinMode(2,INPUT_PULLUP);
	pinMode(3,INPUT_PULLUP);
	pinMode(4,INPUT_PULLUP);
	pinMode(6,INPUT_PULLUP);
	pinMode(11,INPUT_PULLUP);
	pinMode(12,INPUT_PULLUP);
	pinMode(13,INPUT_PULLUP);
	pinMode(17,INPUT_PULLUP);
	pinMode(18,INPUT_PULLUP);
	pinMode(19,INPUT_PULLUP);

	pinMode(7,INPUT_PULLUP);
	pinMode(8,INPUT_PULLUP);
	//pinMode(9,INPUT);
	//pinMode(10,INPUT);
	pinMode(14,INPUT);
	pinMode(15,INPUT);
	pinMode(16,INPUT);
	pinMode(21,INPUT);
	pinMode(22,INPUT);
	pinMode(23,INPUT);
	
	bounceDr.attach(_pinDr);
	bounceDr.interval(1000);
	bounceDu.attach(_pinDu);
	bounceDu.interval(1000);
	bounceDl.attach(_pinDl);
	bounceDl.interval(1000);
	bounceDd.attach(_pinDd);
	bounceDd.interval(1000);
}
void readButtons(){
	btn.A = !digitalRead(_pinA);
	btn.B = !digitalRead(_pinB);
	btn.X = !digitalRead(_pinXSwappable);
	btn.Y = !digitalRead(_pinYSwappable);
	btn.Z = !digitalRead(_pinZSwappable);
	btn.S = !digitalRead(_pinS);
	btn.L = !digitalRead(_pinL);
	btn.R = !digitalRead(_pinR);
	btn.Du = !digitalRead(_pinDu);
	btn.Dd = !digitalRead(_pinDd);
	btn.Dl = !digitalRead(_pinDl);
	btn.Dr = !digitalRead(_pinDr);
	
	bounceDr.update();
	bounceDu.update();
	bounceDl.update();
	bounceDd.update();
	
	if(bounceDr.fell()){
		if(_currentCalStep == -1){
			_calAStick = false;
		}
		_currentCalStep ++;
	}
	else if(bounceDl.fell()){
		if(_currentCalStep == -1){
			_calAStick = true;
		}
		_currentCalStep ++;
	}
	else if(bounceDu.fell()){
		adjustSnapback(btn.Cy);
	}
	else if(bounceDd.fell()){
		readJumpConfig();
	}
	/* 
	bool dPad = (btn.Dl || btn.Dr);
	
	if(dPad && !_lastDPad){
		_dPadSince = millis();
		_watchingDPad = true;
	}
	else if(dPad && _watchingDPad){
		int startTimer = millis()- _dPadSince;
		if(startTimer > 1000){
			if(_currentCalStep == -1){
				if(btn.Dl){
					_calAStick = true;
				}
				else{
					_calAStick = false;
				}
			}
			_currentCalStep ++;
			_watchingDPad = false;
			Serial.println("calibrating");
			Serial.println(_currentCalStep);
		}
	}
	_lastDPad = dPad; */
}
void adjustSnapback(int cStick){
	Serial.println("adjusting snapback filtering");
	if(cStick > 127+50){
		_ADCVarSlow = _ADCVarSlow*1.1;
		Serial.print("filtering increased to:");
		Serial.println(_ADCVarSlow);
	}
	else if(cStick < 127-50){
		_ADCVarSlow = _ADCVarSlow*0.9;
		Serial.print("filtering decreased to:");
		Serial.println(_ADCVarSlow);
	}
	if(_ADCVarSlow >_ADCVarMax){
		_ADCVarSlow = _ADCVarMax;
	}
	else if(_ADCVarSlow < _ADCVarMin){
		_ADCVarSlow = _ADCVarMin;
	}
	setADCVar(&_aADCVar, &_bADCVar, _ADCVarSlow);
	EEPROM.put(_eepromADCVar,_ADCVarSlow);
}
void readJumpConfig(){
	Serial.print("setting jump to: ");
	if(!digitalRead(_pinX)){
		_jumpConfig = 1;
		Serial.println("X<->Z");
	}
	else if(!digitalRead(_pinY)){
		_jumpConfig = 2;
		Serial.println("Y<->Z");
	}
	else{
		Serial.println("normal");
		_jumpConfig = 0;
	}
	EEPROM.put(_eepromJump,_jumpConfig);
	setJump(_jumpConfig);
}
void setJump(int jumpConfig){
	switch(jumpConfig){
			case 1:
				_pinZSwappable = _pinX;
				_pinXSwappable = _pinZ;
				_pinYSwappable = _pinY;
				break;				
			case 2:
				_pinZSwappable = _pinY;
				_pinXSwappable = _pinX;
				_pinYSwappable = _pinZ;
				break;
			default:
				_pinZSwappable = _pinZ;
				_pinXSwappable = _pinX;
				_pinYSwappable = _pinY;
	}
}
void setADCVar(float* aADCVar,float* bADCVar, float ADCVarSlow){
	*aADCVar = (_ADCVarFast - ADCVarSlow)/_x6;
	*bADCVar = ADCVarSlow;
}
void readSticks(){
	//read the L and R sliders
	btn.La = adc->adc0->analogRead(_pinLa)>>4;
	btn.Ra = adc->adc0->analogRead(_pinRa)>>4;
	
	//read the C stick
	//btn.Cx = adc->adc0->analogRead(pinCx)>>4;
	//btn.Cy = adc->adc0->analogRead(pinCy)>>4;
	
	//read the analog stick, scale it down so that we don't get huge values when we linearize
	_aStickX = adc->adc0->analogRead(_pinAx)/4096.0;
	_aStickY = adc->adc0->analogRead(_pinAy)/4096.0;
	
	//read the c stick, scale it down so that we don't get huge values when we linearize 
	_cStickX = adc->adc0->analogRead(_pinCx)/4096.0;
	_cStickY = adc->adc0->analogRead(_pinCy)/4096.0;
	
	//create the measurement vector to be used in the kalman filter
	VectorXf xZ(1);
	VectorXf yZ(1);
	
	//linearize the analog stick inputs by multiplying by the coefficients found during calibration (3rd order fit)
	//store in the measurement vectors
	//xZ << (_aFitCoeffsX[0]*(_aStickX*_aStickX*_aStickX) + _aFitCoeffsX[1]*(_aStickX*_aStickX) + _aFitCoeffsX[2]*_aStickX + _aFitCoeffsX[3]);
	//yZ << (_aFitCoeffsY[0]*(_aStickY*_aStickY*_aStickY) + _aFitCoeffsY[1]*(_aStickY*_aStickY) + _aFitCoeffsY[2]*_aStickY + _aFitCoeffsY[3]);
	xZ << linearize(_aStickX,_aFitCoeffsX);
	yZ << linearize(_aStickY,_aFitCoeffsY);
	
	//float posCx = (_cFitCoeffsX[0]*(_cStickX*_cStickX*_cStickX) + _cFitCoeffsX[1]*(_cStickX*_cStickX) + _cFitCoeffsX[2]*_cStickX + _cFitCoeffsX[3]);
	//float posCy = (_aFitCoeffsY[1]*(_cStickY*_cStickY*_cStickY) + _aFitCoeffsY[1]*(_cStickY*_cStickY) + _aFitCoeffsY[2]*_cStickY + _aFitCoeffsY[3]);
  float posCx = linearize(_cStickX,_cFitCoeffsX);
	float posCy = linearize(_cStickY,_cFitCoeffsY);
	
	//Run the kalman filter to eliminate snapback
	runKalman(xZ,yZ);
/* 	Serial.println();
	Serial.print(xZ[0]);
	Serial.print(",");
	Serial.print(yZ[0]);
	Serial.print(",");
	Serial.print(_xState[0]);
	Serial.print(",");
	Serial.print(_yState[0]);
 */
	float posAx;
	float posAy;
	//float posCx;
	//float posCy;
	int regions;
	if(_notched){
		regions = (_calibrationPointsNotched-1);
	}
	else{
		regions = (_calibrationPoints-1);
	}
	notchRemap(_xState[0],_yState[0], &posAx,  &posAy, _aAffineCoeffs, _aBoundaryAngles,regions);
	notchRemap(posCx,posCy, &posCx,  &posCy, _cAffineCoeffs, _cBoundaryAngles,_calibrationPoints-1);
	
//	if((xState[1] < 0.1) && (xState[1] > -0.1)){
//			btn.Ax = (uint8_t) xState[0];
//	}
//	
//	if((_yState[1] < 0.1) && (_yState[1] > -0.1)){
//			btn.Ay = (uint8_t) _yState[0];
//	}

	//assign the remapped values to the button struct
	btn.Ax = (uint8_t) (posAx+127.5);
	btn.Ay = (uint8_t) (posAy+127.5);
	btn.Cx = (uint8_t) (posCx+127.5);
	btn.Cy = (uint8_t) (posCy+127.5);
}
/*******************
	notchRemap
	Remaps the stick position using affine transforms generated from the notch positions
*******************/
void notchRemap(float xIn, float yIn, float* xOut, float* yOut, float affineCoeffs[][6], float regionAngles[], int regions){
	//determine the angle between the x unit vector and the current position vector
	float angle = atan2f(yIn,xIn);
	
	//unwrap the angle based on the first region boundary
	if(angle < regionAngles[0]){
		angle += M_PI*2;
	}
	
	//go through the region boundaries from lowest angle to highest, checking if the current position vector is in that region
	//if the region is not found then it must be between the first and the last boundary, ie the last region
	//we check GATE_REGIONS*2 because each notch has its own very small region we use to make notch values more consistent
	int region = regions*2-1;
	for(int i = 1; i < regions*2; i++){
		if(angle < regionAngles[i]){
			region = i-1;
			break;
		}
	}

	//Apply the affine transformation using the coefficients found during calibration
	*xOut = affineCoeffs[region][0]*xIn + affineCoeffs[region][1]*yIn + affineCoeffs[region][2];
	*yOut = affineCoeffs[region][3]*xIn + affineCoeffs[region][4]*yIn + affineCoeffs[region][5];
}
/*******************
	setPole
	takes the values that have been put into the button struct and translates them in the serial commands ready
	to be sent to the gamecube/wii
*******************/
void setPole(){
	for(int i = 0; i < 8; i++){
		//we don't want to send data while we're updating the stick and button states, so we turn off interrupts
		noInterrupts();
		//write all of the data in the button struct (taken from the dogebawx project, thanks to GoodDoge)
		for(int j = 0; j < 4; j++){
			//this could probably be done better but we need to take 2 bits at a time to put into one serial byte
			//for details on this read here: http://www.qwertymodo.com/hardware-projects/n64/n64-controller
			int these2bits = (btn.arr[i]>>(6-j*2)) & 3;
			switch(these2bits){
				case 0:
				pollResponse[(i<<2)+j] = 0x08;
				break;
				case 1:
				pollResponse[(i<<2)+j] = 0xE8;
				break;
				case 2:
				pollResponse[(i<<2)+j] = 0x0F;
				break;
				case 3:
				pollResponse[(i<<2)+j] = 0xEF;
				break;
			}
		}
		//turn the interrupt back on so we can start communicating again
		interrupts();
	}
}
/*******************
	communicate
	try to communicate with the gamecube/wii
*******************/
void communicate(){
	//Serial.println("communicating");
	//clear any commands from the last write, once the write queue is empty then any new data is from the master
	//this is needed because the RX pin is connected to the TX pin through the diode
	while(Serial2.available() && (_writeQueue > 0)){
		Serial2.read();
		_writeQueue --;
	}
	
	//check if a full byte is available from the master yet, if its not the exit
	if (Serial2.available()){
		//wait for the first 5 bytes of the command to arrive
		//we need to do this because the interrupt is not necissarily fast enough, so we wait here for all the data so we can respond as soon as possible
		while(Serial2.available()<CMD_LENGTH_SHORT){}
		//read those 5 bytes
		//may be possible to speed things up by reading just 4 bytes and then clearing the last one after
		for(int i = 0; i <CMD_LENGTH_SHORT; i++){
			cmd[i] = Serial2.read();
		}

		//parse first 4 bytes of the command, we don't care about the rest of it
		//the mapping here is a little strange because of how we are using the serial connection, see this for details:
		//http://www.qwertymodo.com/hardware-projects/n64/n64-controller
		for(int i = 0; i <CMD_LENGTH_SHORT-1; i++){
			switch(cmd[i]){
			case 0x08:
				cmdByte = (cmdByte<<2);
				break;
			case 0xE8:
				cmdByte = (cmdByte<<2)+1;
				break;
			case 0x0F:
				cmdByte = (cmdByte<<2)+2;
				break;
			case 0xEF:
				cmdByte = (cmdByte<<2)+3;
				break;
			default:
				//got garbage data or a stop bit where it shouldn't be
				cmdByte = -1;
			}
		}
	//print the command byte over the USB serial  for debugging
	//Serial.println(cmdByte,HEX);
	
	//decide what to do based on the command
	switch(cmdByte){
		//this is the poll command, it will be what is sent continually after a connection is made
		case 0x40:
			//the poll command is longer, but we don't care about any of the other data
			//wait until we get a stop bit, then we know we can start sending data again
		  while(Serial2.read() != 0xFF){}
			//write the pre-prepared poll response out byte by byte
		  for(int i = 0; i <POLL_LENGTH; i++){
				Serial2.write(pollResponse[i]);
				//print to USB serial for debugging
				//Serial.print(pollResponse[i],HEX);
		  }
			//set the write queue so that we will ignore all the data we are sending out
			_writeQueue = POLL_LENGTH;
		  break;
		case 0x00:
			//this is the probe command, its what the master will send out continually when nothing is connected
			//write the pre-prepared probe resonse out byte by byte
		  for(int i = 0; i <CMD_LENGTH_LONG; i++){
				Serial2.write(probeResponse[i]);
		  }
			//set the write queue so that we will ignore all the data we are sending out
			_writeQueue = CMD_LENGTH_LONG;
		  break;
		case 0x41:
			//this is the origin command, it gets sent out a few times after a connection is made before switching to polling, may be related to zeroing the analog sticks and triggers?
		  for(int i = 0; i <ORIGIN_LENGTH; i++){
				Serial2.write(originResponse[i]);
		  }
			//set the write queue so that we will ignore all the data we are sending out
			_writeQueue = ORIGIN_LENGTH;
		  break;
		default:
		  //got something strange, try waiting for a stop bit to syncronize
		  while(Serial2.read() != 0xFF){}
			_writeQueue = 0;
	  }
	}
}
/*******************
	calibrate
	run the calibration procedure
*******************/
void calibrate(bool aStick){
	if(aStick){
		if(_notched){
			int calSteps = (_calibrationPointsNotched-1)*2;
			if(collectCalPoints(true,_aStickX,_aStickY,calSteps,_currentCalStep,_tempCalPointsX,_tempCalPointsY,_cleanedPointsX,_cleanedPointsY)){
				stickCal(_cleanedPointsX,_cleanedPointsY,true,_aFitCoeffsX,_aFitCoeffsY,_aAffineCoeffs,_aBoundaryAngles);
				EEPROM.put(_eepromAPointsX,_cleanedPointsX);
				EEPROM.put(_eepromAPointsY,_cleanedPointsY);
				_currentCalStep = -1;
			}
		}
		else{
			int calSteps = (_calibrationPoints-1)*2;
			if(collectCalPoints(true,_aStickX,_aStickY,calSteps,_currentCalStep,_tempCalPointsX,_tempCalPointsY,_cleanedPointsX,_cleanedPointsY)){
				stickCal(_cleanedPointsX,_cleanedPointsY,false,_aFitCoeffsX,_aFitCoeffsY,_aAffineCoeffs,_aBoundaryAngles);
				EEPROM.put(_eepromAPointsX,_cleanedPointsX);
				EEPROM.put(_eepromAPointsY,_cleanedPointsY);
				_currentCalStep = -1;
			}
		}
	}
	else{
		int calSteps = (_calibrationPoints-1)*2;
		if(collectCalPoints(false,_cStickX,_cStickY,calSteps,_currentCalStep,_tempCalPointsX,_tempCalPointsY,_cleanedPointsX,_cleanedPointsY)){
			stickCal(_cleanedPointsX,_cleanedPointsY,false,_cFitCoeffsX,_cFitCoeffsY,_cAffineCoeffs,_cBoundaryAngles);
			EEPROM.put(_eepromCPointsX,_cleanedPointsX);
			EEPROM.put(_eepromCPointsY,_cleanedPointsY);
			_currentCalStep = -1;
		}
	}
}
int collectCalPoints(bool aStick, float valX, float valY, int numberOfPoints, int currentStep, float calPointsX[], float calPointsY[],float cleanedPointsX[],float cleanedPointsY[]){

	if(currentStep >= (numberOfPoints)){
		Serial.println("finished collecting points");
		Serial.println("The raw points are (x,y):");
		for(int i = 0; i<(numberOfPoints); i++){
			Serial.print(calPointsX[i]);
			Serial.print(",");
			Serial.println(calPointsY[i]);
		}
		cleanedPointsX[0] = 0;
		cleanedPointsY[0] = 0;
		//generate the set of points that will be used for calibration
		//the first is an average of all the origin points collected
		//the remainng (either 8 or 16 points are the notches)
		for(int i = 0; i < numberOfPoints/2; i++){
			//add the origin values toe the first x,y point
			cleanedPointsX[0] += calPointsX[i*2];
			cleanedPointsY[0] += calPointsY[i*2];
			
			//set the notch point
			cleanedPointsX[i+1] = calPointsX[i*2+1];
			cleanedPointsY[i+1] = calPointsY[i*2+1];
		}
		

		//divide by the total number of calibration steps/2 to get the average origin value
		cleanedPointsX[0] = cleanedPointsX[0]/((float)numberOfPoints/2.0);
		cleanedPointsY[0] = cleanedPointsY[0]/((float)numberOfPoints/2.0);
		
		Serial.println("The collected points are after averaging the origin (x,y):");
		for(int i = 0; i<(numberOfPoints/2+1); i++){
			Serial.print(cleanedPointsX[i]);
			Serial.print(",");
			Serial.println(cleanedPointsY[i]);
		}
		currentStep = _notCalibrating;
		return 1;
	}
	//if not then store the next point
	else{
		
		calPointsX[currentStep] = valX;
		calPointsY[currentStep] = valY;
		
		//Serial.println("The collected points are for this step are (x,y):");
		//Serial.print(calPointsX[currentStep]);
		//Serial.print(",");
		//Serial.println(calPointsY[currentStep]);
			
		uint8_t thiGuideX;
		uint8_t thiGuideY;
		if(currentStep%2){
			thiGuideX = (uint8_t) (100*cosf((currentStep-1)/((float)numberOfPoints)*M_PI*2)+127.5);
			thiGuideY = (uint8_t) (100*sinf((currentStep-1)/((float)numberOfPoints)*M_PI*2)+127.5);
		}
		else{
			thiGuideX = 127;
			thiGuideY = 127;
		}
		
/* 		Serial.println("the guide values for this step are:");
			Serial.print(currentStep);
			Serial.print(",");
			Serial.print(currentStep%2);
			Serial.print(",");
			Serial.print(thiGuideX);
			Serial.print(",");
			Serial.println(thiGuideY); */
			
		if(aStick){
			btn.Cx = thiGuideX;
			btn.Cy = thiGuideY;
		}
		else{
			btn.Ax = thiGuideX;
			btn.Ay = thiGuideY;
		}
		return 0;
	}
	return -1;
}
/*******************
	aStickCal
	calibrate the grey analog stick so that its response will be linear and aligned with the controllers notches
	takes a set of cleaned points, the number of points, outputs the fit coefficients, affine transform coefficients, and region boundary angles
	expects that the cleaned points are going around counterclockwise starting from the 3 oclock position
*******************/
void stickCal(float cleanedPointsX[],float cleanedPointsY[], bool notched,float fitCoeffsX[],float fitCoeffsY[], float affineTransCoeffs[][6], float boundaryAngles[]){
	Serial.println("begining stick calibration");
	//create the variables we will need throughout
	int pointsCount;
	double fitPointsX[5];
	double fitPointsY[5];
	float notchPointsX[_calibrationPointsNotched];
	float notchPointsY[_calibrationPointsNotched];
	
	
	//generate all the notched/not notched specific cstick values we will need
	if(notched){
		Serial.println("this controller is notched");
		pointsCount = _calibrationPointsNotched;
		fitPointsX[0] = cleanedPointsX[8+1];
		fitPointsX[1] = (cleanedPointsX[6+1] + cleanedPointsX[10+1])/2.0;
		fitPointsX[2] = cleanedPointsX[0];
		fitPointsX[3] = (cleanedPointsX[2+1] + cleanedPointsX[14+1])/2.0;
		fitPointsX[4] = cleanedPointsX[0+1];
		
		fitPointsY[0] = cleanedPointsY[12+1];
		fitPointsY[1] = (cleanedPointsY[10+1] + cleanedPointsY[14+1])/2.0;
		fitPointsY[2] = cleanedPointsY[0];
		fitPointsY[3] = (cleanedPointsY[6+1] + cleanedPointsY[2+1])/2.0;
		fitPointsY[4] = cleanedPointsY[4+1];
		
		notchPointsX[0] = 0;
		notchPointsX[1] = 100;
		notchPointsX[2] = 95.393920141;
		notchPointsX[3] = 70.7106781187;
		notchPointsX[4] = 30;
		notchPointsX[5] = 0;
		notchPointsX[6] = -30;
		notchPointsX[7] = -70.7106781187;
		notchPointsX[8] = -95.393920141;
		notchPointsX[9] = -100;
		notchPointsX[10] = -95.393920141;
		notchPointsX[11] = -70.7106781187;
		notchPointsX[12] = -30;
		notchPointsX[13] = 0;
		notchPointsX[14] = 30;
		notchPointsX[15] = 70.7106781187;
		notchPointsX[16] = 95.393920141;
		
    notchPointsY[0] = 0;
		notchPointsY[1] = 0;
		notchPointsY[2] = 30;
		notchPointsY[3] = 70.7106781187;
		notchPointsY[4] = 95.393920141;
		notchPointsY[5] = 100;
		notchPointsY[6] = 95.393920141;
		notchPointsY[7] = 70.7106781187;
		notchPointsY[8] = 30;
		notchPointsY[9] = 0;
		notchPointsY[10] = -30;
		notchPointsY[11] = -67.5;
		notchPointsY[12] = -95.393920141;
		notchPointsY[13] = -100;
		notchPointsY[14] = -95.393920141;
		notchPointsY[15] = -67.5;
		notchPointsY[16] = -30;
	}
	else{
		Serial.println("this controller is not notched");
		pointsCount = _calibrationPoints;
				//generate the gate notch points that we will map to

		fitPointsX[0] = cleanedPointsX[4+1];
		fitPointsX[1] = (cleanedPointsX[3+1] + cleanedPointsX[5+1])/2.0;
		fitPointsX[2] = cleanedPointsX[0];
		fitPointsX[3] = (cleanedPointsX[1+1] + cleanedPointsX[7+1])/2.0;
		fitPointsX[4] = cleanedPointsX[0+1];
		
		fitPointsY[0] = cleanedPointsY[6+1];
		fitPointsY[1] = (cleanedPointsY[5+1] + cleanedPointsY[7+1])/2.0;
		fitPointsY[2] = cleanedPointsY[0];
		fitPointsY[3] = (cleanedPointsY[3+1] + cleanedPointsY[1+1])/2.0;
		fitPointsY[4] = cleanedPointsY[2+1];
		
		notchPointsX[0] = 0;
		notchPointsX[1] = 100;
		notchPointsX[2] = 70.7106781187;
		notchPointsX[3] = 0;
		notchPointsX[4] = -70.7106781187;
		notchPointsX[5] = -100;
		notchPointsX[6] = -70.7106781187;
		notchPointsX[7] = 0;
		notchPointsX[8] = 70.7106781187;
		
    notchPointsY[0] = 0;
		notchPointsY[1] = 0;
		notchPointsY[2] = 70.7106781187;
		notchPointsY[3] = 100;
		notchPointsY[4] = 70.7106781187;
		notchPointsY[5] = 0;
		notchPointsY[6] = -67.5;
		notchPointsY[7] = -100;
		notchPointsY[8] = -67.5;
	}
	
	//////determine the coefficients needed to linearize the stick
	//create the expected output, what we want our curve to be fit too
	double x_output[5] = {27.5,56.7893218813,127.5,198.210678119,227.5};
	double y_output[5] = {27.5,56.7893218813,127.5,198.210678119,227.5};
	
	Serial.println("The fit input points are (x,y):");
	for(int i = 0; i < 5; i++){
		Serial.print(fitPointsX[i]);
		Serial.print(",");
		Serial.println(fitPointsY[i]);
	}
	
	Serial.println("The corresponding fit output points are (x,y):");
	for(int i = 0; i < 5; i++){
		Serial.print(x_output[i]);
		Serial.print(",");
		Serial.println(y_output[i]);
	}
	
	//perform the curve fit, order is 3
	double tempCoeffsX[_fitOrder+1];
	double tempCoeffsY[_fitOrder+1];

	fitCurve(_fitOrder, 5, fitPointsX, x_output, _fitOrder+1,  tempCoeffsX);
	fitCurve(_fitOrder, 5, fitPointsY, y_output, _fitOrder+1, tempCoeffsY);
	
		//write these coefficients to the array that was passed in, this is our first output
	for(int i = 0; i < (_fitOrder+1); i++){
		fitCoeffsX[i] = tempCoeffsX[i];
		fitCoeffsY[i] = tempCoeffsY[i];
	}
	
	//we will now take out the offset, making the range -100 to 100 isntead of 28 to 228
	//calculate the offset
	float xZeroError = linearize((float)fitPointsX[2],fitCoeffsX);
	float yZeroError = linearize((float)fitPointsY[2],fitCoeffsY);
	
	//Adjust the fit so that the stick zero position is 0
	fitCoeffsX[3] = fitCoeffsX[3] - xZeroError;
	fitCoeffsY[3] = fitCoeffsY[3] - yZeroError;
	
	Serial.println("The fit coefficients are  are (x,y):");
	for(int i = 0; i < 4; i++){
		Serial.print(fitCoeffsX[i]);
		Serial.print(",");
		Serial.println(fitCoeffsY[i]);
	}
	//linearize the calibration points in preperation for mapping
	float linearizedX[pointsCount];
	float linearizedY[pointsCount];
	for(int i = 0; i<pointsCount;i++){
		linearizedX[i] = linearize(cleanedPointsX[i],fitCoeffsX);
		linearizedY[i] = linearize(cleanedPointsY[i],fitCoeffsY);
	}
	
	Serial.println("The linearized calibration points are (x,y):");
	for(int i = 0; i < pointsCount; i++){
		Serial.print(linearizedX[i]);
		Serial.print(",");
		Serial.println(linearizedY[i]);
	}

	
	//generate the smaller sub regions that will allow the stick to 'snap' into position at a notch
	//these will be the inputs of the mapping
	int pointsCountSnap = (pointsCount-1)*2+1;
	float linearizedSnapX[pointsCountSnap];
	float linearizedSnapY[pointsCountSnap];
	//these will be the outputs of the mapping
	float notchPointsSnapX[pointsCountSnap];
	float notchPointsSnapY[pointsCountSnap];
	
	//the origin will be used as one of the 3 points for all regions, set it now
	linearizedSnapX[0] = linearizedX[0];
	linearizedSnapY[0] = linearizedY[0];
	notchPointsSnapX[0] = notchPointsX[0];
	notchPointsSnapY[0] = notchPointsY[0];
	
	//itterate throught the rest of the notches, creating each sub region by rotating the points slightly CW or CCW around the origin
	for(int i = 1; i < (pointsCount);i++){
		linearizedSnapX[(i*2)-1] = cosf(-_marginAngle)*linearizedX[i] - sinf(-_marginAngle)*linearizedY[i];
		linearizedSnapY[(i*2)-1] = sinf(-_marginAngle)*linearizedX[i] + cosf(-_marginAngle)*linearizedY[i];
		notchPointsSnapX[(i*2)-1] = cosf(-_tightAngle)*notchPointsX[i] - sinf(-_tightAngle)*notchPointsY[i];
		notchPointsSnapY[(i*2)-1] = sinf(-_tightAngle)*notchPointsX[i] + cosf(-_tightAngle)*notchPointsY[i];
		
		linearizedSnapX[i*2] = cosf(_marginAngle)*linearizedX[i] - sinf(_marginAngle)*linearizedY[i];
		linearizedSnapY[i*2] = sinf(_marginAngle)*linearizedX[i] + cosf(_marginAngle)*linearizedY[i];
		notchPointsSnapX[i*2] = cosf(_tightAngle)*notchPointsX[i] - sinf(_tightAngle)*notchPointsY[i];
		notchPointsSnapY[i*2] = sinf(_tightAngle)*notchPointsX[i] + cosf(_tightAngle)*notchPointsY[i];

	}
	
	Serial.println("The subnotch calibration points are (x,y):");
	for(int i = 0; i < pointsCountSnap; i++){
		Serial.print(linearizedSnapX[i]);
		Serial.print(",");
		Serial.println(linearizedSnapY[i]);
	}
	
		Serial.println("The and their correspoinding goal points are (x,y):");
	for(int i = 0; i < pointsCountSnap; i++){
		Serial.print(notchPointsSnapX[i]);
		Serial.print(",");
		Serial.println(notchPointsSnapY[i]);
	}
	
	//perform the notch calibration using all the points generated above
	notchCalibrate(linearizedSnapX,linearizedSnapY,notchPointsSnapX,notchPointsSnapY,pointsCountSnap-1,affineTransCoeffs,boundaryAngles);
	
}
float linearize(float point, float coefficients[0]){
	return (coefficients[0]*(point*point*point) + coefficients[1]*(point*point) + coefficients[2]*point + coefficients[3]);
}
void runKalman(VectorXf& xZ,VectorXf& yZ){
	//Serial.println("Running Kalman");
	
	unsigned int thisMicros = micros();
	float dT = (thisMicros-_lastMicros)/1000.0;
	_lastMicros = thisMicros;
	//Serial.print("the loop time is: ");
	//Serial.println(dT);
	
	MatrixXf Fmat(2,2);
	Fmat << 1,dT-_damping/2*dT*dT,
			 0,1-_damping*dT;

  float R2 = xZ[0]*xZ[0]+yZ[0]*yZ[0];
  //float R2 = (xZ[0]-128.5)*(xZ[0]-128.5);
  if(R2 > 10000){
    R2 = 10000;
  }
  float accelVar = _aAccelVar*(R2*R2*R2) + _bAccelVar;
	
	MatrixXf Q(2,2);
  //Serial.print(accelVar,10);
  //Serial.print(',');
	Q << (dT*dT*dT*dT/4), (dT*dT*dT/2),
			 (dT*dT*dT/2), (dT*dT);
	Q = Q * accelVar;
	
	MatrixXf sharedH(1,2);
	sharedH << 1,0;
	
	//Serial.println('H');
	//print_mtxf(H)
	
	MatrixXf xR(1,1);
	//MatrixXf yR(1,1);
	

  xR << _aADCVar*(R2*R2*R2) + _bADCVar;

  //Serial.println(xR(0,0),10);
	//dT = micros()-lastMicros;
	//Serial.println(dT);
	//print_mtxf(_xP);
	kPredict(_xState,Fmat,_xP,Q);
	//dT = micros()-lastMicros;
	//Serial.println(dT);
	
	kPredict(_yState,Fmat,_yP,Q);
	//dT = micros()-lastMicros;
	//Serial.println(dT);
	
	kUpdate(_xState,xZ,_xP,sharedH,xR);
	//dT = micros()-lastMicros;
	//Serial.println(dT);
	
	kUpdate(_yState,yZ,_yP,sharedH,xR);
	//dT = micros()-lastMicros;
	//Serial.println(dT);
}
void kPredict(VectorXf& X, MatrixXf& F, MatrixXf& P, MatrixXf& Q){
	//Serial.println("Predicting Kalman");
	
	X = F*X;
	P = F*P*F.transpose() + Q;
	
}
void kUpdate(VectorXf& X, VectorXf& Z, MatrixXf& P, MatrixXf& H,  MatrixXf& R){
//void kUpdate(VectorXf& X, float measX, MatrixXf& P, MatrixXf& H,  MatrixXf& R){
	//Serial.println("Updating Kalman");
	
	int sizeState = X.size();
	//int sizeMeas = Z.size();
	MatrixXf A(1,2);
	A = P*H.transpose();
	MatrixXf B(1,1);
	B = H*A+R;
	MatrixXf K(2,1);
	K = A*B.inverse();
	//print_mtxf(K);
	//K = MatrixXf::Identity(sizeState,sizeState);
	MatrixXf C(1,1); 
	//C = Z - H*X;
	X = X + K*(Z - H*X);
	//X = X + K*(measX-X[0]);
	
	MatrixXf D = MatrixXf::Identity(sizeState,sizeState) - K*H;
	P = D*P*D.transpose() + K*R*K.transpose();
	
	//print_mtxf(P);
}
void notchCalibrate(float* xIn, float* yIn, float* xOut, float* yOut, int regions, float allAffineCoeffs[][6], float regionAngles[]){
	for(int i = 1; i <= regions; i++){
      Serial.print("calibrating region: ");
      Serial.println(i);
      
		MatrixXf pointsIn(3,3);

    MatrixXf pointsOut(3,3);
    
    if(i == regions){
      Serial.println("final region");
      pointsIn << xIn[0],xIn[i],xIn[1],
                yIn[0],yIn[i],yIn[1],
                1,1,1;
      pointsOut << xOut[0],xOut[i],xOut[1],
                   yOut[0],yOut[i],yOut[1],
                   1,1,1;
    }
    else{
		pointsIn << xIn[0],xIn[i],xIn[i+1],
								yIn[0],yIn[i],yIn[i+1],
								1,1,1;
		pointsOut << xOut[0],xOut[i],xOut[i+1],
								 yOut[0],yOut[i],yOut[i+1],
								 1,1,1;
    }
    
    Serial.println("In points:");
    print_mtxf(pointsIn);
    Serial.println("Out points:");
    print_mtxf(pointsOut);
    
		MatrixXf A(3,3);
    
		A = pointsOut*pointsIn.inverse();
    //A = pointsOut.colPivHouseholderQr().solve(pointsIn);
   

    Serial.println("The transform matrix is:");
    print_mtxf(A);
    
    Serial.println("The affine transform coefficients for this region are:");
    
		for(int j = 0; j <2;j++){
			for(int k = 0; k<3;k++){
				allAffineCoeffs[i-1][j*3+k] = A(j,k);
				Serial.print(allAffineCoeffs[i-1][j*3+k]);
				Serial.print(",");
			}
		}
		
		Serial.println();
		Serial.println("The angle defining this  regions is:");
		regionAngles[i-1] = atan2f((yIn[i]-yIn[0]),(xIn[i]-xIn[0]));
		//unwrap the angles so that the first has the smallest value
		if(regionAngles[i-1] < regionAngles[0]){
			regionAngles[i-1] += M_PI*2;
		}
		Serial.println(regionAngles[i-1]);
	}
}
void print_mtxf(const Eigen::MatrixXf& X){
   int i, j, nrow, ncol;
   nrow = X.rows();
   ncol = X.cols();
   Serial.print("nrow: "); Serial.println(nrow);
   Serial.print("ncol: "); Serial.println(ncol);       
   Serial.println();
   for (i=0; i<nrow; i++)
   {
       for (j=0; j<ncol; j++)
       {
           Serial.print(X(i,j), 6);   // print 6 decimal places
           Serial.print(", ");
       }
       Serial.println();
   }
   Serial.println();
}
