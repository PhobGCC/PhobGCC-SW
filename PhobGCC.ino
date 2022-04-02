//This software uses bits of code from GoodDoge's Dogebawx project, which was the initial starting point: https://github.com/DogeSSBM/DogeBawx

#include <math.h>
#include <curveFitting.h>
#include <EEPROM.h>
#include <eigen.h>
#include <Eigen/LU>
#include <ADC.h>
#include <VREF.h>
#include <Bounce2.h>
#include "TeensyTimerTool.h"

//Uncomment the appropriate include line for your hardware.
//#include "src/Phob1_0Teensy3_2.h"
//#include "src/Phob1_1Teensy3_2.h"

using namespace Eigen;

#define CMD_LENGTH_SHORT 5
#define CMD_LENGTH_LONG 13
#define PROBE_LENGTH 13
#define ORIGIN_LENGTH 41
#define POLL_LENGTH 33
#define CALIBRATION_POINTS 17

TeensyTimerTool::OneShotTimer timer1;

////Serial bitbanging settings
const int _fastBaud = 1250000;
//const int _slowBaud = 1000000;
const int _slowBaud = 1000000;
const int _fastDivider = (((F_CPU * 2) + ((_fastBaud) >> 1)) / (_fastBaud));
const int _slowDivider = (((F_CPU * 2) + ((_slowBaud) >> 1)) / (_slowBaud));
const int _fastBDH = (_fastDivider >> 13) & 0x1F;
const int _slowBDH = (_slowDivider >> 13) & 0x1F;
const int _fastBDL = (_fastDivider >> 5) & 0xFF;
const int _slowBDL = (_slowDivider >> 5) & 0xFF;
const int _fastC4 = _fastDivider & 0x1F;
const int _slowC4 = _slowDivider & 0x1F;

//defining control configuration
int _pinZSwappable = _pinZ;
int _pinXSwappable = _pinX;
int _pinYSwappable = _pinY;
int _jumpConfig = 0;
int _lConfig = 0;
int _rConfig = 0;
int _lTrigger = 0;
int _rTrigger = 1;
bool _changeTrigger = true;

///// Values used for dealing with snapback in the Kalman Filter, a 6th power relationship between distance to center and ADC/acceleration variance is used, this was arrived at by trial and error

float _velDampMin = 0.125;
float _velDampMax = .5;


//New snapback Kalman filter parameters.
struct FilterGains {
    //What's the max stick distance from the center
    float maxStick;
    //filtered velocity terms
    //how fast the filtered velocity falls off in the absence of stick movement.
    //Probably don't touch this.
    float xVelDecay;//0.1 default for 1.2ms timesteps, larger for bigger timesteps
    float yVelDecay;
    //how much the current position disagreement impacts the filtered velocity.
    //Probably don't touch this.
    float xVelPosFactor;//0.01 default for 1.2ms timesteps, larger for bigger timesteps
    float yVelPosFactor;
    //how much to ignore filtered velocity when computing the new stick position.
    //DO CHANGE THIS
    //Higher gives shorter rise times and slower fall times (more pode, less snapback)
    float xVelDamp;//0.125 default for 1.2ms timesteps, smaller for bigger timesteps
    float yVelDamp;
    //speed and accel thresholds below which we try to follow the stick better
    //These may need tweaking according to how noisy the signal is
    //If it's noisier, we may need to add additional filtering
    //If the timesteps are *really small* then it may need to be increased to get
    //  above the noise floor. Or some combination of filtering and playing with
    //  the thresholds.
    float velThresh;//1 default for 1.2ms timesteps, larger for bigger timesteps
    float accelThresh;//5 default for 1.2ms timesteps, larger for bigger timesteps
};
FilterGains _gains {//these values are actually timestep-compensated for in runKalman
    .maxStick = 100,
    .xVelDecay = 0.1,
    .yVelDecay = 0.1,
    .xVelPosFactor = 0.01,
    .yVelPosFactor = 0.01,
    .xVelDamp = 0.125,
    .yVelDamp = 0.125,
    .velThresh = 1.0,
    .accelThresh = 3.0
};

//////values used to determine how much large of a region will count as being "in a notch"

const float _marginAngle = 1.50/100.0; //angle range(+/-) in radians that will be collapsed down to the ideal angle
const float _tightAngle = 0.1/100.0;//angle range(+/-) in radians that the margin region will be collapsed down to, found that having a small value worked better for the transform than 0

//////values used for calibration
const int _noOfNotches = 16;
const int _noOfCalibrationPoints = _noOfNotches * 2;
float _ADCScale = 1;
float _ADCScaleFactor = 1;
const int _notCalibrating = -1;
const float _maxStickAngle = 0.67195176201;
bool	_calAStick = true; //determines which stick is being calibrated (if false then calibrate the c-stick)
bool _advanceCal = false;
bool _advanceCalPressed = false;
bool _undoCal = false;
bool _undoCalPressed = false;
int _currentCalStep; //keeps track of which caliblration step is active, -1 means calibration is not running
bool _notched = false; //keeps track of whether or not the controller has firefox notches
const int _calibrationPoints = _noOfNotches+1; //number of calibration points for the c-stick and a-stick for a controller without notches
float _cleanedPointsX[_noOfNotches+1]; //array to hold the x coordinates of the stick positions for calibration
float _cleanedPointsY[_noOfNotches+1]; //array to hold the y coordinates of the stick positions for calibration
float _notchPointsX[_noOfNotches+1]; //array to hold the x coordinates of the notches for calibration
float _notchPointsY[_noOfNotches+1]; //array to hold the x coordinates of the notches for calibration
const float _cDefaultCalPointsX[_noOfCalibrationPoints] = {0.507073712,0.9026247224,0.5072693007,0.5001294236,0.5037118952,0.8146074226,0.5046028951,0.5066508636,0.5005339326,0.5065670067,0.5006805723,0.5056853599,0.5058308703,0.1989667596,0.5009560613,0.508400395,0.507729394,0.1003568119,0.5097473849,0.5074989796,0.5072406293,0.2014042034,0.5014653263,0.501119675,0.502959011,0.5032433665,0.5018446562,0.5085523857,0.5099732513,0.8100862401,0.5089320995,0.5052066109};
const float _cDefaultCalPointsY[_noOfCalibrationPoints] = {0.5006151799,0.5025356503,0.501470528,0.5066983468,0.5008275958,0.8094667357,0.5008874968,0.5079207909,0.5071239815,0.9046004275,0.5010136589,0.5071086316,0.5058914031,0.8076523013,0.5078213507,0.5049117887,0.5075638281,0.5003774649,0.504562192,0.50644895,0.5074859854,0.1983865682,0.5074515232,0.5084323402,0.5015846608,0.1025902875,0.5043605453,0.5070589342,0.5073953693,0.2033337702,0.5005351734,0.5056548782};
const float _aDefaultCalPointsX[_noOfCalibrationPoints] = {0.3010610568,0.3603937084,0.3010903951,0.3000194135,0.3005567843,0.3471911134,0.3006904343,0.3009976295,0.3000800899,0.300985051,0.3001020858,0.300852804,0.3008746305,0.2548450139,0.3001434092,0.3012600593,0.3011594091,0.2400535218,0.3014621077,0.3011248469,0.3010860944,0.2552106305,0.3002197989,0.3001679513,0.3004438517,0.300486505,0.3002766984,0.3012828579,0.3014959877,0.346512936,0.3013398149,0.3007809916};
const float _aDefaultCalPointsY[_noOfCalibrationPoints] = {0.300092277,0.3003803475,0.3002205792,0.301004752,0.3001241394,0.3464200104,0.3001331245,0.3011881186,0.3010685972,0.3606900641,0.3001520488,0.3010662947,0.3008837105,0.3461478452,0.3011732026,0.3007367683,0.3011345742,0.3000566197,0.3006843288,0.3009673425,0.3011228978,0.2547579852,0.3011177285,0.301264851,0.3002376991,0.2403885431,0.3006540818,0.3010588401,0.3011093054,0.2555000655,0.300080276,0.3008482317};
const float _notchAngleDefaults[_noOfNotches] = {0,M_PI/8.0,M_PI*2/8.0,M_PI*3/8.0,M_PI*4/8.0,M_PI*5/8.0,M_PI*6/8.0,M_PI*7/8.0,M_PI*8/8.0,M_PI*9/8.0,M_PI*10/8.0,M_PI*11/8.0,M_PI*12/8.0,M_PI*13/8.0,M_PI*14/8.0,M_PI*15/8.0};
const float _notchRange[_noOfNotches] = {0,M_PI*1/16.0,M_PI/16.0,M_PI*1/16.0,0,M_PI*1/16.0,M_PI/16.0,M_PI*1/16.0,0,M_PI*1/16.0,M_PI/16.0,M_PI*1/16.0,0,M_PI*1/16.0,M_PI/16.0,M_PI*1/16.0};
const int _notchStatusDefaults[_noOfNotches] = {3,1,2,1,3,1,2,1,3,1,2,1,3,1,2,1};
float _aNotchAngles[_noOfNotches] = {0,M_PI/8.0,M_PI*2/8.0,M_PI*3/8.0,M_PI*4/8.0,M_PI*5/8.0,M_PI*6/8.0,M_PI*7/8.0,M_PI*8/8.0,M_PI*9/8.0,M_PI*10/8.0,M_PI*11/8.0,M_PI*12/8.0,M_PI*13/8.0,M_PI*14/8.0,M_PI*15/8.0};
int _aNotchStatus[_noOfNotches] = {3,1,2,1,3,1,2,1,3,1,2,1,3,1,2,1};
float _cNotchAngles[_noOfNotches];
int _cNotchStatus[_noOfNotches] = {3,1,2,1,3,1,2,1,3,1,2,1,3,1,2,1};
const int _cardinalNotch = 3;
const int _secondaryNotch = 2;
const int _tertiaryNotchActive = 1;
const int _tertiaryNotchInactive = 0;
const int _fitOrder = 3; //fit order used in the linearization step
float _aFitCoeffsX[_fitOrder+1]; //coefficients for linearizing the X axis of the a-stick
float _aFitCoeffsY[_fitOrder+1]; //coefficients for linearizing the Y axis of the a-stick
float _cFitCoeffsX[_fitOrder+1]; //coefficients for linearizing the Y axis of the c-stick
float _cFitCoeffsY[_fitOrder+1]; //coefficients for linearizing the Y axis of the c-stick
float _aAffineCoeffs[_noOfNotches][6]; //affine transformation coefficients for all regions of the a-stick
float _cAffineCoeffs[_noOfNotches][6]; //affine transformation coefficients for all regions of the c-stick
float _aBoundaryAngles[_noOfNotches]; //angles at the boundaries between regions of the a-stick
float _cBoundaryAngles[_noOfNotches]; //angles at the boundaries between regions of the c-stick
float _tempCalPointsX[(_noOfNotches)*2]; //temporary storage for the x coordinate points collected during calibration before the are cleaned and put into _cleanedPointsX
float _tempCalPointsY[(_noOfNotches)*2]; //temporary storage for the y coordinate points collected during calibration before the are cleaned and put into _cleanedPointsY

//index values to store data into eeprom
const int _bytesPerFloat = 4;
const int _eepromAPointsX = 0;
const int _eepromAPointsY = _eepromAPointsX+_noOfCalibrationPoints*_bytesPerFloat;
const int _eepromCPointsX = _eepromAPointsY+_noOfCalibrationPoints*_bytesPerFloat;
const int _eepromCPointsY = _eepromCPointsX+_noOfCalibrationPoints*_bytesPerFloat;
const int _eepromxVelDamp = _eepromCPointsY+_noOfCalibrationPoints*_bytesPerFloat;
const int _eepromyVelDamp = _eepromxVelDamp+_bytesPerFloat;
const int _eepromJump = _eepromyVelDamp+_bytesPerFloat;
const int _eepromANotchAngles = _eepromJump+_bytesPerFloat;
const int _eepromCNotchAngles = _eepromANotchAngles+_noOfNotches*_bytesPerFloat;
const int _eepromLToggle = _eepromCNotchAngles+_noOfNotches*_bytesPerFloat;
const int _eepromRToggle = _eepromLToggle+_bytesPerFloat;

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
float _posALastX;
float _aStickY;
float _posALastY;
float _cStickX;
float _cStickY;

volatile int _writeQueue = 0;



unsigned int _lastMicros;
float _dT;
bool _running = false;

//The median filter can be either length 3, 4, or 5.
#define MEDIANLEN 5
//Or just comment this define to disable it entirely.
//#define USEMEDIAN
float _xPosList[MEDIANLEN];//for median filtering
float _yPosList[MEDIANLEN];//for median filtering
unsigned int _xMedianIndex;
unsigned int _yMedianIndex;

//new kalman filter state variables
float _xPos;//input of kalman filter
float _yPos;//input of kalman filter
float _xPosFilt;//output of kalman filter
float _yPosFilt;//output of kalman filter
float _xVel;
float _yVel;
float _xVelFilt;
float _yVelFilt;


const char probeResponse[PROBE_LENGTH] = {
	0x08,0x08,0x0F,0xE8,
	0x08,0x08,0x08,0x08,
	0x08,0x08,0x08,0xEF,
	0xFF};
volatile char pollResponse[POLL_LENGTH] = {
0x08,0x08,0x08,0x08,
0x0F,0x08,0x08,0x08,
0xE8,0xEF,0xEF,0xEF,
0xE8,0xEF,0xEF,0xEF,
0xE8,0xEF,0xEF,0xEF,
0xE8,0xEF,0xEF,0xEF,
0x08,0xEF,0xEF,0x08,
0x08,0xEF,0xEF,0x08,
0xFF};
const char originResponse[ORIGIN_LENGTH] = {
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
int cmdByte;
volatile char _bitCount = 0;
volatile bool _probe = false;
volatile bool _pole = false;
volatile int _commStatus = 0;
static int _commIdle = 0;
static int _commRead = 1;
static int _commPoll = 2;
static int _commWrite = 3;

void setup() {

	//start USB serial
	Serial.begin(57600);
	//Serial.println("Software version 0.17 (hopefully Phobos remembered to update this message)");
	Serial.println("This is not a stable version");
	delay(1000);

	readEEPROM();

	//set some of the unused values in the message response
	btn.errS = 0;
	btn.errL = 0;
	btn.orig = 0;
	btn.high = 1;

	_currentCalStep = _notCalibrating;

    for (int i = 0; i < MEDIANLEN; i++){
        _xPosList[i] = 0;
        _yPosList[i] = 0;
    }
    _xMedianIndex = 0;
    _yMedianIndex = 0;

    _xPos = 0;
    _yPos = 0;
    _xPosFilt = 0;
    _yPosFilt = 0;
    _xVel = 0;
    _yVel = 0;
    _xVelFilt = 0;
    _yVelFilt = 0;

	_lastMicros = micros();

    setPinModes();

    boardSpecificSetup(adc, _ADCScale, _ADCScaleFactor);

	//_slowBaud = findFreq();
  //serialFreq = 950000;
	//Serial.print("starting hw serial at freq:");
	//Serial.println(_slowBaud);
	//start hardware serial
	Serial2.begin(_slowBaud);
	//UART1_C2 &= ~UART_C2_RE;
	//attach the interrupt which will call the communicate function when the data line transitions from high to low

	timer1.begin(communicate);
	//timer2.begin(checkCmd);
	//timer3.begin(writePole);
	digitalWriteFast(12,HIGH);
	//ARM_DEMCR |= ARM_DEMCR_TRCENA;
	//ARM_DWT_CTRL |= ARM_DWT_CTRL_CYCCNTENA;
	attachInterrupt(_pinRX, bitCounter, FALLING);
	NVIC_SET_PRIORITY(IRQ_PORTC, 0);
}

void loop() {
	//read the controllers buttons
	readButtons();
	//read the analog inputs
	readSticks();
	//check to see if we are calibrating
	if(_currentCalStep >= 0){
		if(_calAStick){
			adjustNotch(_currentCalStep,_dT,btn.Y,btn.X,true,_aNotchAngles,_aNotchStatus);
		}
		else{
			adjustNotch(_currentCalStep,_dT,btn.Y,btn.X,false,_cNotchAngles,_cNotchStatus);
		}

	}
	//check if we should be reporting values yet
	if(btn.B && !_running){
		Serial.println("Starting to report values");
		_running=true;
	}
	//update the pole message so new data will be sent to the gamecube
	//if(_running){
	setPole();
	//}

}
void readEEPROM(){
	//get the jump setting
	EEPROM.get(_eepromJump, _jumpConfig);
	if(std::isnan(_jumpConfig)){
		_jumpConfig = 0;
	}
	setJump(_jumpConfig);

	//get the L setting
	EEPROM.get(_eepromLToggle, _lConfig);
	if(std::isnan(_lConfig)) {
		_lConfig = 0;
	}
	setLRToggle(_lTrigger, _lConfig, !_changeTrigger);

	//get the R setting
	EEPROM.get(_eepromRToggle, _rConfig);
	if(std::isnan(_rConfig)) {
		_rConfig = 0;
	}
	setLRToggle(_rTrigger, _rConfig, !_changeTrigger);

  //get the x-axis velocity dampening
  EEPROM.get(_eepromxVelDamp, _gains.xVelDamp);
  Serial.print("the xVelDamp value from eeprom is:");
  Serial.println(_gains.xVelDamp);
  if(std::isnan(_gains.xVelDamp)){
		_gains.xVelDamp = _velDampMin;
		Serial.print("the xVelDamp value was adjusted to:");
		Serial.println(_gains.xVelDamp);
	}
  if (_gains.xVelDamp > _velDampMax) {
    _gains.xVelDamp = _velDampMax;
  } else if (_gains.xVelDamp < _velDampMin) {
    _gains.xVelDamp = _velDampMin;
  }

  //get the y-axis velocity dampening
  EEPROM.get(_eepromyVelDamp, _gains.yVelDamp);
  Serial.print("the yVelDamp value from eeprom is:");
  Serial.println(_gains.yVelDamp);
  if(std::isnan(_gains.yVelDamp)){
		_gains.yVelDamp = _velDampMin;
		Serial.print("the yVelDamp value was adjusted to:");
		Serial.println(_gains.yVelDamp);
	}
  if (_gains.yVelDamp > _velDampMax) {
    _gains.yVelDamp = _velDampMax;
  } else if (_gains.yVelDamp < _velDampMin) {
    _gains.yVelDamp = _velDampMin;
  }

	//get the calibration points collected during the last A stick calibration
	EEPROM.get(_eepromAPointsX, _tempCalPointsX);
	EEPROM.get(_eepromAPointsY, _tempCalPointsY);
	EEPROM.get(_eepromANotchAngles, _aNotchAngles);
	cleanCalPoints(_tempCalPointsX,_tempCalPointsY,_aNotchAngles,_cleanedPointsX,_cleanedPointsY,_notchPointsX,_notchPointsY);
	Serial.println("calibration points cleaned");
	linearizeCal(_cleanedPointsX,_cleanedPointsY,_cleanedPointsX,_cleanedPointsY,_aFitCoeffsX,_aFitCoeffsY);
	Serial.println("A stick linearized");
	notchCalibrate(_cleanedPointsX,_cleanedPointsY, _notchPointsX, _notchPointsY, _noOfNotches, _aAffineCoeffs, _aBoundaryAngles);
	//stickCal(_cleanedPointsX,_cleanedPointsY,_aNotchAngles,_aFitCoeffsX,_aFitCoeffsY,_aAffineCoeffs,_aBoundaryAngles);

	//get the calibration points collected during the last A stick calibration
	EEPROM.get(_eepromCPointsX, _tempCalPointsX);
	EEPROM.get(_eepromCPointsY, _tempCalPointsY);
	EEPROM.get(_eepromCNotchAngles, _cNotchAngles);
	cleanCalPoints(_tempCalPointsX,_tempCalPointsY,_cNotchAngles,_cleanedPointsX,_cleanedPointsY,_notchPointsX,_notchPointsY);
	Serial.println("calibration points cleaned");
	linearizeCal(_cleanedPointsX,_cleanedPointsY,_cleanedPointsX,_cleanedPointsY,_cFitCoeffsX,_cFitCoeffsY);
	Serial.println("C stick linearized");
	notchCalibrate(_cleanedPointsX,_cleanedPointsY, _notchPointsX, _notchPointsY, _noOfNotches, _cAffineCoeffs, _cBoundaryAngles);
	//stickCal(_cleanedPointsX,_cleanedPointsY,_cNotchAngles,_cFitCoeffsX,_cFitCoeffsY,_cAffineCoeffs,_cBoundaryAngles);
}
void resetDefaults(){
	Serial.println("RESETTING ALL DEFAULTS");

	_jumpConfig = 0;
	setJump(_jumpConfig);
	EEPROM.put(_eepromJump,_jumpConfig);

	_lConfig = 0;
	_rConfig = 0;
	EEPROM.put(_eepromLToggle, _lConfig);
	EEPROM.put(_eepromRToggle, _rConfig);
	setLRToggle(_lTrigger, _lConfig, !_changeTrigger);
	setLRToggle(_rTrigger, _rConfig, !_changeTrigger);

  _gains.xVelDamp = _velDampMin;
  EEPROM.put(_eepromxVelDamp,_gains.xVelDamp);
  _gains.yVelDamp = _velDampMin;
  EEPROM.put(_eepromyVelDamp,_gains.yVelDamp);

	for(int i = 0; i < _noOfNotches; i++){
		_aNotchAngles[i] = _notchAngleDefaults[i];
		_cNotchAngles[i] = _notchAngleDefaults[i];
	}
	EEPROM.put(_eepromANotchAngles,_aNotchAngles);
	EEPROM.put(_eepromCNotchAngles,_cNotchAngles);

	for(int i = 0; i < _noOfCalibrationPoints; i++){
		_tempCalPointsX[i] = _aDefaultCalPointsX[i];
		_tempCalPointsY[i] = _aDefaultCalPointsY[i];
	}
	EEPROM.put(_eepromAPointsX,_tempCalPointsX);
	EEPROM.put(_eepromAPointsY,_tempCalPointsY);

	Serial.println("A calibration points stored in EEPROM");
	cleanCalPoints(_tempCalPointsX,_tempCalPointsY,_aNotchAngles,_cleanedPointsX,_cleanedPointsY,_notchPointsX,_notchPointsY);
	Serial.println("A calibration points cleaned");
	linearizeCal(_cleanedPointsX,_cleanedPointsY,_cleanedPointsX,_cleanedPointsY,_aFitCoeffsX,_aFitCoeffsY);
	Serial.println("A stick linearized");
	notchCalibrate(_cleanedPointsX,_cleanedPointsY, _notchPointsX, _notchPointsY, _noOfNotches, _aAffineCoeffs, _aBoundaryAngles);

	for(int i = 0; i < _noOfCalibrationPoints; i++){
		_tempCalPointsX[i] = _cDefaultCalPointsX[i];
		_tempCalPointsY[i] = _cDefaultCalPointsY[i];
	}
	EEPROM.put(_eepromCPointsX,_tempCalPointsX);
	EEPROM.put(_eepromCPointsY,_tempCalPointsY);

	Serial.println("C calibration points stored in EEPROM");
	cleanCalPoints(_tempCalPointsX,_tempCalPointsY,_cNotchAngles,_cleanedPointsX,_cleanedPointsY,_notchPointsX,_notchPointsY);
	Serial.println("C calibration points cleaned");
	linearizeCal(_cleanedPointsX,_cleanedPointsY,_cleanedPointsX,_cleanedPointsY,_cFitCoeffsX,_cFitCoeffsY);
	Serial.println("C stick linearized");
	notchCalibrate(_cleanedPointsX,_cleanedPointsY, _notchPointsX, _notchPointsY, _noOfNotches, _cAffineCoeffs, _cBoundaryAngles);

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


	//check the dpad buttons to change the controller settings
	if(bounceDr.fell()){
		if(_currentCalStep == -1){
			Serial.println("Calibrating the C stick");
			_calAStick = false;
			_currentCalStep ++;
			_advanceCal = true;
		}
	}
	else if(bounceDl.fell()){
		if(_currentCalStep == -1){
			if(btn.S == true){
				resetDefaults();
			}
			else{
				Serial.println("Calibrating the A stick");
				_calAStick = true;
				_currentCalStep ++;
				_advanceCal = true;
			}
		}
	}
	else if(bounceDu.fell()){
		adjustSnapback(btn.Cx,btn.Cy);
	}
	else if(bounceDd.fell()){
		if(btn.L) {
			setLRToggle(_lTrigger, 0, _changeTrigger);
		} else if(btn.R) {
			setLRToggle(_rTrigger, 0, _changeTrigger);
		} else {
			readJumpConfig();
		}

	}
	//Undo Calibration using B-button
	if(btn.B && _undoCal && !_undoCalPressed) {
		_undoCalPressed = true;
		if(_currentCalStep % 2 == 0 && _currentCalStep != 32 && _currentCalStep != 0) {
			_currentCalStep --;
			_currentCalStep --;
		}
	} else if(!btn.B) {
		_undoCalPressed = false;
	}

	//Advance Calibration Using A-button
	if(btn.A && _advanceCal && !_advanceCalPressed){
		_advanceCalPressed = true;
		if (!_calAStick){
			collectCalPoints(_calAStick, _currentCalStep,_tempCalPointsX,_tempCalPointsY);
			_currentCalStep ++;
			if(_currentCalStep >= 2) {
				_undoCal = true;
			} else {
				_undoCal = false;
			}
			if(_currentCalStep >= _noOfNotches*2){
				Serial.println("finished collecting the calibration points for the C stick");
				EEPROM.put(_eepromCPointsX,_tempCalPointsX);
				EEPROM.put(_eepromCPointsY,_tempCalPointsY);
				EEPROM.put(_eepromCNotchAngles,_cNotchAngles);
				Serial.println("calibration points stored in EEPROM");
				cleanCalPoints(_tempCalPointsX,_tempCalPointsY,_cNotchAngles,_cleanedPointsX,_cleanedPointsY,_notchPointsX,_notchPointsY);
				Serial.println("calibration points cleaned");
				linearizeCal(_cleanedPointsX,_cleanedPointsY,_cleanedPointsX,_cleanedPointsY,_cFitCoeffsX,_cFitCoeffsY);
				Serial.println("C stick linearized");
				notchCalibrate(_cleanedPointsX,_cleanedPointsY, _notchPointsX, _notchPointsY, _noOfNotches, _cAffineCoeffs, _cBoundaryAngles);
				_currentCalStep = -1;
				_advanceCal = false;
			}
		}
		else if (_calAStick){
			collectCalPoints(_calAStick, _currentCalStep,_tempCalPointsX,_tempCalPointsY);
			_currentCalStep ++;
			if(_currentCalStep >= 2) {
				_undoCal = true;
			} else {
				_undoCal = false;
			}
			if(_currentCalStep >= _noOfNotches*2){
				Serial.println("finished collecting the calibration points for the A stick");
				EEPROM.put(_eepromAPointsX,_tempCalPointsX);
				EEPROM.put(_eepromAPointsY,_tempCalPointsY);
				EEPROM.put(_eepromANotchAngles,_aNotchAngles);
				Serial.println("calibration points stored in EEPROM");
				cleanCalPoints(_tempCalPointsX,_tempCalPointsY,_aNotchAngles,_cleanedPointsX,_cleanedPointsY,_notchPointsX,_notchPointsY);
				Serial.println("calibration points cleaned");
				linearizeCal(_cleanedPointsX,_cleanedPointsY,_cleanedPointsX,_cleanedPointsY,_aFitCoeffsX,_aFitCoeffsY);
				Serial.println("A stick linearized");
				notchCalibrate(_cleanedPointsX,_cleanedPointsY, _notchPointsX, _notchPointsY, _noOfNotches, _aAffineCoeffs, _aBoundaryAngles);
				_currentCalStep = -1;
				_advanceCal = false;
			}
		}
	} else if(!btn.A) {
		_advanceCalPressed = false;
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
void adjustSnapback(int cStickX, int cStickY){
	Serial.println("adjusting snapback filtering");
	if(cStickX > 127+50){
		_gains.xVelDamp = _gains.xVelDamp*1.2599;
		Serial.print("X filtering increased to:");
		Serial.println(_gains.xVelDamp);
	}
	else if(cStickX < 127-50){
		_gains.xVelDamp = _gains.xVelDamp*0.7937;
		Serial.print("X filtering decreased to:");
		Serial.println(_gains.xVelDamp);
	}
	if(_gains.xVelDamp > _velDampMax){
		_gains.xVelDamp = _velDampMax;
	}
	else if(_gains.xVelDamp < _velDampMin){
		_gains.xVelDamp = _velDampMin;
	}

	if(cStickY > 127+50){
		_gains.yVelDamp = _gains.yVelDamp*1.2599;
		Serial.print("Y filtering increased to:");
		Serial.println(_gains.yVelDamp);
	}
	else if(cStickY < 127-50){
		_gains.yVelDamp = _gains.yVelDamp*0.7937;
		Serial.print("Y filtering decreased to:");
		Serial.println(_gains.yVelDamp);
	}
	if(_gains.yVelDamp >_velDampMax){
		_gains.yVelDamp = _velDampMax;
	}
	else if(_gains.yVelDamp < _velDampMin){
		_gains.yVelDamp = _velDampMin;
	}

  float xVarDisplay = 3 * (log(_gains.xVelDamp / 0.125) / log(2));
  float yVarDisplay = 3 * (log(_gains.yVelDamp / 0.125) / log(2));

	Serial.println("Var display results");
		Serial.println(xVarDisplay);
	Serial.println(yVarDisplay);


	btn.Cx = (uint8_t) (xVarDisplay + 127.5);
	btn.Cy = (uint8_t) (yVarDisplay + 127.5);

	setPole();

	int startTime = millis();
	int delta = 0;
	while(delta < 2000){
		delta = millis() - startTime;
	}

	EEPROM.put(_eepromxVelDamp,_gains.xVelDamp);
	EEPROM.put(_eepromyVelDamp,_gains.yVelDamp);
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
/*
* setLRToggle handles the current state of the L and R Triggers and whether or not they should be enabled or not.
* int targetTrigger handles identifying the trigger, L = 0  and R = 1.
* if it is 0, it should read out an actual analog value. If it is 1, it shouldn't.
* config handles incoming values from the EEPROM. takes the state and sets it.
* changeTrigger handles whether or not the current configuration of the targetTrigger should be swapped or not.
* TODO: Create variables for the states and triggers.
*/
void setLRToggle(int targetTrigger, int config, bool changeTrigger) {
	if(changeTrigger) {
		if(targetTrigger == _lTrigger) {
			if(_lConfig == 0) {
				_lConfig = 1;
			} else {
				_lConfig = 0;
			}
			EEPROM.put(_eepromLToggle, _lConfig);
		} else {
			if(_rConfig == 0) {
				_rConfig = 1;
			} else {
				_rConfig = 0;
			}
			EEPROM.put(_eepromRToggle, _rConfig);
		}
	} else {
		if(targetTrigger == _lTrigger) {
			_lConfig = config;
			EEPROM.put(_eepromLToggle, _lConfig);
		} else {
			_rConfig = config;
			EEPROM.put(_eepromRToggle, _rConfig);
		}
	}
}
void readSticks(){
#ifdef USEADCSCALE
    _ADCScale = _ADCScale*0.999 + _ADCScaleFactor/adc->adc1->analogRead(ADC_INTERNAL_SOURCE::VREF_OUT);
#endif
    // otherwise _ADCScale is 1

	//read the analog stick, scale it down so that we don't get huge values when we linearize
	_aStickX = adc->adc0->analogRead(_pinAx)/4096.0*_ADCScale;
	_aStickY = adc->adc0->analogRead(_pinAy)/4096.0*_ADCScale;


	//read the L and R sliders
	if(_lConfig == 0) {
			btn.La = adc->adc0->analogRead(_pinLa)>>4;
	} else {
			btn.La = (uint8_t) 0;
	}

	if(_rConfig == 0) {
		btn.Ra = adc->adc0->analogRead(_pinRa)>>4;
	} else {
			btn.Ra = (uint8_t) 0;
	}

	//read the C stick
	//btn.Cx = adc->adc0->analogRead(pinCx)>>4;
	//btn.Cy = adc->adc0->analogRead(pinCy)>>4;


	//read the c stick, scale it down so that we don't get huge values when we linearize
	_cStickX = (_cStickX + adc->adc0->analogRead(_pinCx)/4096.0)*0.5;
	_cStickY = (_cStickY + adc->adc0->analogRead(_pinCy)/4096.0)*0.5;

	//create the measurement value to be used in the kalman filter
	float xZ;
	float yZ;

	//linearize the analog stick inputs by multiplying by the coefficients found during calibration (3rd order fit)
	//store in the measurement vectors
	//xZ << (_aFitCoeffsX[0]*(_aStickX*_aStickX*_aStickX) + _aFitCoeffsX[1]*(_aStickX*_aStickX) + _aFitCoeffsX[2]*_aStickX + _aFitCoeffsX[3]);
	//yZ << (_aFitCoeffsY[0]*(_aStickY*_aStickY*_aStickY) + _aFitCoeffsY[1]*(_aStickY*_aStickY) + _aFitCoeffsY[2]*_aStickY + _aFitCoeffsY[3]);
	xZ = linearize(_aStickX,_aFitCoeffsX);
	yZ = linearize(_aStickY,_aFitCoeffsY);

	//float posCx = (_cFitCoeffsX[0]*(_cStickX*_cStickX*_cStickX) + _cFitCoeffsX[1]*(_cStickX*_cStickX) + _cFitCoeffsX[2]*_cStickX + _cFitCoeffsX[3]);
	//float posCy = (_aFitCoeffsY[1]*(_cStickY*_cStickY*_cStickY) + _aFitCoeffsY[1]*(_cStickY*_cStickY) + _aFitCoeffsY[2]*_cStickY + _aFitCoeffsY[3]);
  float posCx = linearize(_cStickX,_cFitCoeffsX);
	float posCy = linearize(_cStickY,_cFitCoeffsY);

    //Run a median filter to reduce noise
#ifdef USEMEDIAN
    runMedian(xZ, _xPosList, _xMedianIndex);
    runMedian(yZ, _yPosList, _yMedianIndex);
#endif

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

	notchRemap(_xPosFilt, _yPosFilt, &posAx,  &posAy, _aAffineCoeffs, _aBoundaryAngles,_noOfNotches);
	notchRemap(posCx,posCy, &posCx,  &posCy, _cAffineCoeffs, _cBoundaryAngles,_noOfNotches);

	//assign the remapped values to the button struct
	if(_running){
		btn.Ax = (uint8_t) (posAx+127.5);
		btn.Ay = (uint8_t) (posAy+127.5);
		btn.Cx = (uint8_t) (posCx+127.5);
		btn.Cy = (uint8_t) (posCy+127.5);
	}
	else
	{
		btn.Ax = 127;
		btn.Ay = 127;
		btn.Cx = 127;
		btn.Cy = 127;
	}

	_posALastX = posAx;
	_posALastY = posAy;
	//Serial.println();
	//Serial.print(_dT/16.7);
	//Serial.print(",");
	//Serial.print(xZ[0],8);
	//Serial.print(",");
	//Serial.print(_velFilterX*10,8);
	//Serial.print(",");
	//Serial.print(yZ[0],8);
	//Serial.print(",");
	//Serial.print((posAx+127.5),8);
	//Serial.print(",");
	//Serial.print((posAy+127.5),8);
	//Serial.print(",");
	//Serial.print(btn.Ax);
	//Serial.print(",");
	//Serial.print(btn.Ay);
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
	//int region = regions*2-1;
	int region = regions-1;
	for(int i = 1; i < regions; i++){
		if(angle < regionAngles[i]){
			region = i-1;
			break;
		}
	}

	//Apply the affine transformation using the coefficients found during calibration
	*xOut = affineCoeffs[region][0]*xIn + affineCoeffs[region][1]*yIn + affineCoeffs[region][2];
	*yOut = affineCoeffs[region][3]*xIn + affineCoeffs[region][4]*yIn + affineCoeffs[region][5];

	if((abs(*xOut)<5) && (abs(*yOut)>95)){
		*xOut = 0;
	}
	if((abs(*yOut)<5) && (abs(*xOut)>95)){
		*yOut = 0;
	}
}
/*******************
	setPole
	takes the values that have been put into the button struct and translates them in the serial commands ready
	to be sent to the gamecube/wii
*******************/
void setPole(){
	for(int i = 0; i < 8; i++){
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

	}
}
/*******************
	communicate
	try to communicate with the gamecube/wii
*******************/
void bitCounter(){
	_bitCount ++;
	//digitalWriteFast(12,!(_bitCount%2));
	if(_bitCount == 1){
		timer1.trigger((CMD_LENGTH_SHORT-1)*10);
		_commStatus = _commRead;
	}
}
void communicate(){
	//Serial.println(_commStatus,DEC);
	if(_commStatus == _commRead){
		//digitalWriteFast(12,LOW);
		//Serial.println(Serial2.available(),DEC);
		while(Serial2.available() < (CMD_LENGTH_SHORT-1)){}
			cmdByte = 0;
			for(int i = 0; i < CMD_LENGTH_SHORT-1; i++){
				cmd[i] = Serial2.read();
				//Serial.println(cmd[i],BIN);
				switch(cmd[i]){
				case 0x08:
					cmdByte = (cmdByte<<2);
					break;
				case 0xE8:
					cmdByte = (cmdByte<<2)+1;
					break;
 				case 0xC8:
					cmdByte = (cmdByte<<2)+1;
					break;
				case 0x0F:
					cmdByte = (cmdByte<<2)+2;
					break;
				case 0x0E:
					cmdByte = (cmdByte<<2)+2;
					break;
				case 0xEF:
					cmdByte = (cmdByte<<2)+3;
					break;
				case 0xCF:
					cmdByte = (cmdByte<<2)+3;
					break;
				case 0xEE:
					cmdByte = (cmdByte<<2)+3;
					break;
				case 0xCE:
					cmdByte = (cmdByte<<2)+3;
					break;
				default:
					//got garbage data or a stop bit where it shouldn't be
					Serial.println(cmd[i],BIN);
					cmdByte = -1;
					//Serial.println('o');
				}
				if(cmdByte == -1){
					Serial.println('b');
					break;
				}
			}

		//Serial.println(cmdByte,HEX);
		UART1_BDH = _fastBDH;
		UART1_BDL = _fastBDL;
		UART1_C4 = _fastC4;
		UART1_C2 &= ~UART_C2_RE;

		switch(cmdByte){
		case 0x00:
			//digitalWriteFast(12,LOW);
			timer1.trigger(PROBE_LENGTH*8);
			//Serial2.write(probeResponse,PROBE_LENGTH);
			for(int i = 0; i< PROBE_LENGTH; i++){
				Serial2.write(probeResponse[i]);
			}
			Serial.println("probe");
			_writeQueue = 9+(PROBE_LENGTH-1)*2+1;
			_commStatus = _commWrite;
			//digitalWriteFast(12,HIGH);
		break;
		case 0x41:
			timer1.trigger(ORIGIN_LENGTH*8);
			//Serial2.write(originResponse,ORIGIN_LENGTH);
			for(int i = 0; i< ORIGIN_LENGTH; i++){
				Serial2.write(originResponse[i]);
			}
			Serial.println("origin");
			_writeQueue = 9+(ORIGIN_LENGTH-1)*2+1;
			_commStatus = _commWrite;
		  break;
		case 0x40:
			timer1.trigger(56);
			_commStatus = _commPoll;
			break;
		default:
		  //got something strange, try waiting for a stop bit to syncronize
			//resetFreq();
			digitalWriteFast(12,LOW);
			Serial.println("error");
			Serial.println(_bitCount,DEC);

			UART1_BDH = _slowBDH;
			UART1_BDL = _slowBDL;
			UART1_C4 = _slowC4;
			UART1_C2 |= UART_C2_RE;
			Serial2.clear();

			uint8_t thisbyte = 0;
		  while(thisbyte != 0xFF){
				while(!Serial2.available());
				thisbyte = Serial2.read();
				Serial.println(thisbyte,BIN);
			}
			//Serial2.clear();
			_commStatus = _commIdle;
			_bitCount = 0;
			_writeQueue = 0;
			digitalWriteFast(12,HIGH);
	  }
		//digitalWriteFast(12,HIGH);
	}
	else if(_commStatus == _commPoll){
		digitalWriteFast(12,LOW);
		while(_bitCount<25){}
		//Serial2.write((const char*)pollResponse,POLL_LENGTH);
		for(int i = 0; i< POLL_LENGTH; i++){
			Serial2.write(pollResponse[i]);
		}
		timer1.trigger(135);
		_writeQueue = 25+(POLL_LENGTH-1)*2+1;
		_commStatus = _commWrite;
		//digitalWriteFast(12,HIGH);
	}
	else if(_commStatus == _commWrite){
		//digitalWriteFast(12,LOW);
 		while(_writeQueue > _bitCount){}

		UART1_BDH = _slowBDH;
		UART1_BDL = _slowBDL;
		UART1_C4 = _slowC4;
		UART1_C2 |= UART_C2_RE;

		_bitCount = 0;
		_commStatus = _commIdle;
		_writeQueue = 0;
		digitalWriteFast(12,HIGH);
		Serial2.clear();
	}
	else{
		Serial.println('a');
	}
	//Serial.println(_commStatus,DEC);
}
/*******************
	cleanCalPoints
	take the x and y coordinates and notch angles collected during the calibration procedure, and generate the cleaned x an y stick coordinates and the corresponding x and y notch coordinates
*******************/
void cleanCalPoints(float calPointsX[], float  calPointsY[], float notchAngles[], float cleanedPointsX[], float cleanedPointsY[], float notchPointsX[], float notchPointsY[]){

	Serial.println("The raw calibration points (x,y) are:");
	for(int i = 0; i< _noOfCalibrationPoints; i++){
		Serial.print(calPointsX[i]);
		Serial.print(",");
		Serial.println(calPointsY[i]);
	}

	Serial.println("The notch angles are:");
	for(int i = 0; i< _noOfNotches; i++){
		Serial.println(notchAngles[i]);
	}

	notchPointsX[0] = 0;
	notchPointsY[0] = 0;
	cleanedPointsX[0] = 0;
	cleanedPointsY[0] = 0;

	Serial.println("The notch points are:");
	for(int i = 0; i < _noOfNotches; i++){
			//add the origin values to the first x,y point
			cleanedPointsX[0] += calPointsX[i*2];
			cleanedPointsY[0] += calPointsY[i*2];

			//set the notch point
			cleanedPointsX[i+1] = calPointsX[i*2+1];
			cleanedPointsY[i+1] = calPointsY[i*2+1];

			calcStickValues(notchAngles[i], notchPointsX+i+1, notchPointsY+i+1);
			//notchPointsX[i+1] = ((int)notchPointsX[i+1] + 0.5);
			//notchPointsY[i+1] = ((int)notchPointsY[i+1] + 0.5);
			notchPointsX[i+1] = round(notchPointsX[i+1]);
			notchPointsY[i+1] = round(notchPointsY[i+1]);

			Serial.print(notchPointsX[i+1]);
			Serial.print(",");
			Serial.println(notchPointsY[i+1]);
		}


		//divide by the total number of calibration steps/2 to get the average origin value
		cleanedPointsX[0] = cleanedPointsX[0]/((float)_noOfNotches);
		cleanedPointsY[0] = cleanedPointsY[0]/((float)_noOfNotches);

	for(int i = 0; i < _noOfNotches; i++){
		float deltaX = cleanedPointsX[i+1] - cleanedPointsX[0];
		float deltaY = cleanedPointsY[i+1] - cleanedPointsY[0];
		float mag = sqrt(deltaX*deltaX + deltaY*deltaY);
		if(mag < 0.02){
			int prevIndex = (i-1+_noOfNotches) % _noOfNotches+1;
			int nextIndex = (i+1) % _noOfNotches+1;

			cleanedPointsX[i+1] = (cleanedPointsX[prevIndex] + cleanedPointsX[nextIndex])/2.0;
			cleanedPointsY[i+1] = (cleanedPointsY[prevIndex] + cleanedPointsY[nextIndex])/2.0;

			notchPointsX[i+1] = (notchPointsX[prevIndex] + notchPointsX[nextIndex])/2.0;
			notchPointsY[i+1] = (notchPointsY[prevIndex] + notchPointsY[nextIndex])/2.0;

			Serial.print("no input was found for notch: ");
			Serial.println(i+1);
		}
	}

	Serial.println("The cleaned calibration points are:");
	for(int i = 0; i< (_noOfNotches+1); i++){
		Serial.print(cleanedPointsX[i]);
		Serial.print(",");
		Serial.println(cleanedPointsY[i]);
	}

	Serial.println("The corresponding notch points are:");
	for(int i = 0; i< (_noOfNotches+1); i++){
		Serial.print(notchPointsX[i]);
		Serial.print(",");
		Serial.println(notchPointsY[i]);
	}
}
void adjustNotch(int currentStep, float loopDelta, bool CW, int CCW, bool calibratingAStick, float notchAngles[], int notchStatus[]){
	float X = 0;
	float Y = 0;
	//don't run on center steps
	if(currentStep%2){
		int notchIndex = currentStep/2;
		//Serial.println(notchAngles[notchIndex]);
		if(notchStatus[notchIndex] != _cardinalNotch){
			if(CW){
				notchAngles[notchIndex] += loopDelta*0.00005;
			}
			else if(CCW){
				notchAngles[notchIndex] -= loopDelta*0.00005;
			}
		}
		if(notchAngles[notchIndex] > _notchAngleDefaults[notchIndex]+_notchRange[notchIndex]){
			notchAngles[notchIndex] = _notchAngleDefaults[notchIndex]+_notchRange[notchIndex];
		}
		else if(notchAngles[notchIndex] < _notchAngleDefaults[notchIndex]-_notchRange[notchIndex]){
			notchAngles[notchIndex] = _notchAngleDefaults[notchIndex]-_notchRange[notchIndex];
		}
		calcStickValues(notchAngles[notchIndex], &X, &Y);
	}
	if(calibratingAStick){
		btn.Cx = (uint8_t) (X + 127.5);
		btn.Cy = (uint8_t) (Y + 127.5);
	}
	else{
		btn.Ax = (uint8_t) (X + 127.5);
		btn.Ay = (uint8_t) (Y + 127.5);
	}
}
void collectCalPoints(bool aStick, int currentStep, float calPointsX[], float calPointsY[]){

	Serial.print("Collecting cal point for step: ");
	Serial.println(currentStep);
	float X = 0;
	float Y = 0;

	for(int i = 0; i < 128; i++){
		if(aStick){
#ifdef USEADCSCALE
            _ADCScale = _ADCScale*0.999 + _ADCScaleFactor/adc->adc1->analogRead(ADC_INTERNAL_SOURCE::VREF_OUT);
#endif
            //otherwise _ADCScale is 1
			X += adc->adc0->analogRead(_pinAx)/4096.0*_ADCScale;
			Y += adc->adc0->analogRead(_pinAy)/4096.0*_ADCScale;
		}
		else{
			X += adc->adc0->analogRead(_pinCx)/4096.0;
			Y += adc->adc0->analogRead(_pinCy)/4096.0;
		}
	}

	calPointsX[currentStep] = X/128.0;
	calPointsY[currentStep] = Y/128.0;

	Serial.println("The collected coordinates are: ");
	Serial.println(calPointsX[currentStep],8);
	Serial.println(calPointsY[currentStep],8);
}
/*******************
	linearizeCal
	calibrate a stick so that its response will be linear
	Inputs:
		cleaned points X and Y, (must be 17 points for each of these, the first being the center, the others starting at 3 oclock and going around counterclockwise)
	Outputs:
		linearization fit coefficients X and Y
*******************/
void linearizeCal(float inX[],float inY[],float outX[], float outY[], float fitCoeffsX[],float fitCoeffsY[]){
	Serial.println("beginning linearization");

	//do the curve fit first
	//generate all the notched/not notched specific cstick values we will need

	double fitPointsX[5];
	double fitPointsY[5];

	fitPointsX[0] = inX[8+1];
	fitPointsX[1] = (inX[6+1] + inX[10+1])/2.0;
	fitPointsX[2] = inX[0];
	fitPointsX[3] = (inX[2+1] + inX[14+1])/2.0;
	fitPointsX[4] = inX[0+1];

	fitPointsY[0] = inY[12+1];
	fitPointsY[1] = (inY[10+1] + inY[14+1])/2.0;
	fitPointsY[2] = inY[0];
	fitPointsY[3] = (inY[6+1] + inY[2+1])/2.0;
	fitPointsY[4] = inY[4+1];


	//////determine the coefficients needed to linearize the stick
	//create the expected output, what we want our curve to be fit too
	//this is hard coded because it doesn't depend on the notch adjustments
	double x_output[5] = {27.5,53.2537879754,127.5,201.7462120246,227.5};
	double y_output[5] = {27.5,53.2537879754,127.5,201.7462120246,227.5};

	Serial.println("The fit input points are (x,y):");
	for(int i = 0; i < 5; i++){
		Serial.print(fitPointsX[i],8);
		Serial.print(",");
		Serial.println(fitPointsY[i],8);
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

	Serial.println("The linearized points are:");
	for(int i = 0; i <= _noOfNotches; i++){
		outX[i] = linearize(inX[i],fitCoeffsX);
		outY[i] = linearize(inY[i],fitCoeffsY);
		Serial.print(outX[i],8);
		Serial.print(",");
		Serial.println(outY[i],8);
	}

}

void notchCalibrate(float xIn[], float yIn[], float xOut[], float yOut[], int regions, float allAffineCoeffs[][6], float regionAngles[]){
	for(int i = 1; i <= regions; i++){
      Serial.print("calibrating region: ");
      Serial.println(i);

		MatrixXf pointsIn(3,3);

    MatrixXf pointsOut(3,3);

    if(i == (regions)){
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
float linearize(float point, float coefficients[]){
	return (coefficients[0]*(point*point*point) + coefficients[1]*(point*point) + coefficients[2]*point + coefficients[3]);
}
void runMedian(float &val, float valArray[MEDIANLEN], unsigned int &medianIndex){
    //takes the value, inserts it into the value array, and then
    // writes the median back to the value
    valArray[medianIndex] = val;
    medianIndex = (medianIndex + 1) % MEDIANLEN;

    //We'll hardcode different sort versions according to how long the median is
    //These are derived from RawTherapee's median.h.
#if MEDIANLEN == 3
    val = max(min(valArray[0], valArray[1]), min(valArray[2], max(valArray[0], valArray[1])));
#elif MEDIANLEN == 4
    float maximin = max(min(valArray[0], valArray[1]), min(valArray[2], valArray[3]));
    float minimax = min(max(valArray[0], valArray[1]), max(valArray[2], valArray[3]));
    val = (maximin + minimax) / 2.0f;
#else //MEDIANLEN == 5
    float tmpArray[MEDIANLEN];
    float tmp;
    tmp         = min(valArray[0], valArray[1]);
    tmpArray[1] = max(valArray[0], valArray[1]);
    tmpArray[0] = tmp;
    tmp         = min(valArray[3], valArray[4]);
    tmpArray[4] = max(valArray[3], valArray[4]);
    tmpArray[3] = max(tmpArray[0], tmp);
    tmpArray[1] = min(tmpArray[1], tmpArray[4]);
    tmp         = min(tmpArray[1], valArray[2]);
    tmpArray[2] = max(tmpArray[1], valArray[2]);
    tmpArray[1] = tmp;
    tmp         = min(tmpArray[2], tmpArray[3]);
    val         = max(tmpArray[1], tmp);
#endif
}
void runKalman(const float xZ,const float yZ){
	//Serial.println("Running Kalman");

	//get the time delta since the kalman filter was last run
	unsigned int thisMicros = micros();
	_dT = (thisMicros-_lastMicros)/1000.0;
	_lastMicros = thisMicros;
	//Serial.print("loop time: ");
	//Serial.println(_dT);

    //set up gains according to the time delta.
    //The reference time delta used to tune was 1.2 ms.
    FilterGains g;
    const float timeFactor = _dT / 1.2;
    const float timeDivisor = 1.2 / _dT;
    g.maxStick      = _gains.maxStick*_gains.maxStick;//we actually use the square
    g.xVelDecay     = _gains.xVelDecay      * timeFactor;
    g.yVelDecay     = _gains.yVelDecay      * timeFactor;
    g.xVelPosFactor = _gains.xVelPosFactor  * timeFactor;
    g.yVelPosFactor = _gains.yVelPosFactor  * timeFactor;
    g.xVelDamp      = _gains.xVelDamp       * timeDivisor;
    g.yVelDamp      = _gains.yVelDamp       * timeDivisor;
    g.velThresh     = 1/(_gains.velThresh   * timeFactor);//slight optimization by using the inverse
    g.accelThresh   = 1/(_gains.accelThresh * timeFactor);
    g.velThresh     = g.velThresh*g.velThresh;//square it because it's used squared
    g.accelThresh   = g.accelThresh*g.accelThresh;

    //save previous values of state
    //float _xPos;//input of kalman filter
    //float _yPos;//input of kalman filter
    const float oldXPos = _xPos;
    const float oldYPos = _yPos;
    //float _xPosFilt;//output of kalman filter
    //float _yPosFilt;//output of kalman filter
    const float oldXPosFilt = _xPosFilt;
    const float oldYPosFilt = _yPosFilt;
    //float _xVel;
    //float _yVel;
    const float oldXVel = _xVel;
    const float oldYVel = _yVel;
    //float _xVelFilt;
    //float _yVelFilt;
    const float oldXVelFilt = _xVelFilt;
    const float oldYVelFilt = _yVelFilt;

    //compute new (more trivial) state
    _xPos = xZ;
    _yPos = yZ;
    _xVel = _xPos - oldXPos;
    _yVel = _yPos - oldYPos;
    const float xVelSmooth = 0.5*(_xVel + oldXVel);
    const float yVelSmooth = 0.5*(_yVel + oldYVel);
    const float xAccel = _xVel - oldXVel;
    const float yAccel = _yVel - oldYVel;
    const float oldXPosDiff = oldXPos - oldXPosFilt;
    const float oldYPosDiff = oldYPos - oldYPosFilt;

    //compute stick position exponents for weights
    const float stickDistance2 = min(g.maxStick, _xPos*_xPos + _yPos*_yPos)/g.maxStick;//0-1
    const float stickDistance6 = stickDistance2*stickDistance2*stickDistance2;

    //the current velocity weight for the filtered velocity is the stick r^2
    const float velWeight1 = stickDistance2;
    const float velWeight2 = 1-velWeight1;

    //modified velocity to feed into our kalman filter.
    //We don't actually want an accurate model of the velocity, we want to suppress snapback without adding delay
    //term 1: weight current velocity according to r^2
    //term 2: the previous filtered velocity, weighted the opposite and also set to decay
    //term 3: a corrective factor based on the disagreement between real and filtered position
    _xVelFilt = velWeight1*_xVel + (1-g.xVelDecay)*velWeight2*oldXVelFilt + g.xVelPosFactor*oldXPosDiff;
    _yVelFilt = velWeight1*_yVel + (1-g.yVelDecay)*velWeight2*oldYVelFilt + g.yVelPosFactor*oldYPosDiff;

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
    const float xPosWeightVelAcc = 1 - min(1, xVelSmooth*xVelSmooth*g.velThresh + xAccel*xAccel*g.accelThresh);
    const float xPosWeight1 = max(xPosWeightVelAcc, stickDistance6);
    const float xPosWeight2 = 1-xPosWeight1;
    const float yPosWeightVelAcc = 1 - min(1, yVelSmooth*yVelSmooth*g.velThresh + yAccel*yAccel*g.accelThresh);
    const float yPosWeight1 = max(yPosWeightVelAcc, stickDistance6);
    const float yPosWeight2 = 1-yPosWeight1;

    //In calculating the filtered stick position, we have the following components
    //term 1: current position, weighted according to the above weight
    //term 2: a predicted position based on the filtered velocity and previous filtered position,
    //  with the filtered velocity damped, and the overall term weighted inverse of the previous term
    //term 3: the integral error correction term
    _xPosFilt = xPosWeight1*_xPos +
                xPosWeight2*(oldXPosFilt + (1-g.xVelDamp)*_xVelFilt);
    _yPosFilt = yPosWeight1*_yPos +
                yPosWeight2*(oldYPosFilt + (1-g.yVelDamp)*_yVelFilt);
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
void calcStickValues(float angle, float* x, float* y){
	*x = 100*atan2f((sinf(_maxStickAngle)*cosf(angle)),cosf(_maxStickAngle))/_maxStickAngle;
	*y = 100*atan2f((sinf(_maxStickAngle)*sinf(angle)),cosf(_maxStickAngle))/_maxStickAngle;
}
