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
extern "C" uint32_t set_arm_clock(uint32_t frequency);

//Uncomment the appropriate #include line for your hardware by deleting the two slashes at the beginning of the line.
//#include "src/Phob1_0Teensy3_2.h"          // For PhobGCC board 1.0 with Teensy 3.2
//#include "src/Phob1_0Teensy3_2DiodeShort.h"// For PhobGCC board 1.0 with Teensy 3.2 and the diode shorted
//#include "src/Phob1_1Teensy3_2.h"          // For PhobGCC board 1.1 with Teensy 3.2
//#include "src/Phob1_1Teensy3_2DiodeShort.h"// For PhobGCC board 1.1 with Teensy 3.2 and the diode shorted
//#include "src/Phob1_1Teensy4_0.h"          // For PhobGCC board 1.1 with Teensy 4.0
//#include "src/Phob1_1Teensy4_0DiodeShort.h"// For PhobGCC board 1.1 with Teensy 4.0 and the diode shorted
#include "src/Phob1_2Teensy4_0.h"          // For PhobGCC board 1.2.x with Teensy 4.0

//#define BUILD_RELEASE
#define BUILD_DEV

//This is just an integer.
#define SW_VERSION 26

//#define ENABLE_LED

using namespace Eigen;

TeensyTimerTool::OneShotTimer timer1;

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
}_btn;

struct HardwareButtons{
	uint8_t L;
	uint8_t R;
	uint8_t Z;
	uint8_t X;
	uint8_t Y;
} _hardware;

enum JumpConfig {
	DEFAULT,
	SWAP_XZ,
	SWAP_YZ
};


//defining control configuration
int _pinZSwappable = _pinZ;
int _pinXSwappable = _pinX;
int _pinYSwappable = _pinY;
JumpConfig _jumpConfig = DEFAULT;
const int _jumpConfigMin = 0;
const int _jumpConfigMax = 2;
int _lConfig = 0;
int _rConfig = 0;
const int _triggerConfigMin = 0;
const int _triggerConfigMax = 5;
int _triggerDefault = 0;
int _lTrigger = 0;
int _rTrigger = 1;
bool _changeTrigger = true;
int _cXOffset = 0;
int _cYOffset = 0;
int _cMax = 127;
int _cMin = -127;
int _LTriggerOffset = 49;
int _RTriggerOffset = 49;
const int _triggerMin = 49;
const int _triggerMax = 227;
//rumble config; 0 is off, nonzero is on. Higher values are stronger, max 7
int _rumble = 5;
int calcRumblePower(const int rumble){
	return pow(2.0, 7+((rumble+1)/8.0)); //should be 256 when rumble is 7
}
int _rumblePower = calcRumblePower(_rumble);
const int _rumbleMin = 0;
const int _rumbleMax = 7;
const int _rumbleDefault = 5;
bool _safeMode = true;
bool _autoInit = false;

int trigL,trigR;

///// Values used for adjusting snapback in the CarVac Filter

int _xSnapback = 4;
int _ySnapback = 4;
const float _snapbackMin = 0;
const float _snapbackMax = 10;
const float _snapbackDefault = 4;//0 disables the filter completely, 4 is the default
float velDampFromSnapback(const int snapback){
	return 0.125 * pow(2, (snapback-4)/3.0);//4 should yield 0.125, 10 should yield 0.5, don't care about 0
}

// Values used for dealing with X/Y Smoothing in the CarVac Filter, for ledge-dashing
// also used for C-stick snapback filtering

float _smoothingMin = 0.0;
float _smoothingMax = 0.9;

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
	//This just applies a low-pass filter.
	//The purpose is to provide delay for single-axis ledgedashes.
	//Must be between 0 and 1. Larger = more smoothing and delay.
	float xSmoothing;
	float ySmoothing;
	//Same thing but for C-stick
	float cXSmoothing;
	float cYSmoothing;
};
FilterGains _gains {//these values are actually timestep-compensated for in runKalman
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
FilterGains _g;//this gets filled by recomputeGains();

//////values used to determine how much large of a region will count as being "in a notch"

const float _marginAngle = 1.50/100.0; //angle range(+/-) in radians that will be collapsed down to the ideal angle
const float _tightAngle = 0.1/100.0;//angle range(+/-) in radians that the margin region will be collapsed down to, found that having a small value worked better for the transform than 0

//////values used for calibration
const int _analogCenter = 127;
const int _noOfNotches = 16;
const int _noOfCalibrationPoints = _noOfNotches * 2;
const int _noOfAdjNotches = 12;
float _ADCScale = 1;
float _ADCScaleFactor = 1;
const int _notCalibrating = -1;
const float _maxStickAngle = 0.4886921906;//28 degrees; this is the max angular deflection of the stick.
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
//                                                         right                     notch 1                   up right                  notch 2                   up                        notch 3                   up left                   notch 4                   left                      notch 5                   down left                 notch 6                   down                      notch 7                   down right                notch 8
//                                                         0            1            2            3            4            5            6            7            8            9            10           11           12           13           14           15           16           17           18           19           20           21           22           23           24           25           26           27           28           29           30           31

const float _defaultCalPointsX[_noOfCalibrationPoints] = {0.3010610568,0.3603937084,0.3010903951,0.3000194135,0.3005567843,0.3471911134,0.3006904343,0.3009976295,0.3000800899,0.300985051, 0.3001020858,0.300852804, 0.3008746305,0.2548450139,0.3001434092,0.3012600593,0.3011594091,0.2400535218,0.3014621077,0.3011248469,0.3010860944,0.2552106305,0.3002197989,0.3001679513,0.3004438517,0.300486505, 0.3002766984,0.3012828579,0.3014959877,0.346512936, 0.3013398149,0.3007809916};
const float _defaultCalPointsY[_noOfCalibrationPoints] = {0.300092277, 0.3003803475,0.3002205792,0.301004752, 0.3001241394,0.3464200104,0.3001331245,0.3011881186,0.3010685972,0.3606900641,0.3001520488,0.3010662947,0.3008837105,0.3461478452,0.3011732026,0.3007367683,0.3011345742,0.3000566197,0.3006843288,0.3009673425,0.3011228978,0.2547579852,0.3011177285,0.301264851, 0.3002376991,0.2403885431,0.3006540818,0.3010588401,0.3011093054,0.2555000655,0.300080276, 0.3008482317};
//                                                         right        up          left          down         up right     up left      down left    down right   notch 1      notch 2      notch 3      notch 4      notch 5      notch 6      notch 7      notch 8
const int _calOrder[_noOfCalibrationPoints] =             {0, 1,        8, 9,       16, 17,       24, 25,      4, 5,        12, 13,      20, 21,      28, 29,      2, 3,        6, 7,        10, 11,      14, 15,      18, 19,      22, 23,      26, 27,      30, 31};
//                                                         right        notch 1      up right     notch 2      up           notch 3      up left      notch 4      left         notch 5      down left    notch 6      down         notch 7      down right   notch 8
//                                                         0            1            2            3            4            5            6            7            8            9            10           11           12           13           14           15
const float _notchAngleDefaults[_noOfNotches] =           {0,           M_PI/8.0,    M_PI*2/8.0,  M_PI*3/8.0,  M_PI*4/8.0,  M_PI*5/8.0,  M_PI*6/8.0,  M_PI*7/8.0,  M_PI*8/8.0,  M_PI*9/8.0,  M_PI*10/8.0, M_PI*11/8.0, M_PI*12/8.0, M_PI*13/8.0, M_PI*14/8.0, M_PI*15/8.0};
//const float _notchRange[_noOfNotches] =                   {0,           M_PI*1/16.0, M_PI/16.0,   M_PI*1/16.0, 0,           M_PI*1/16.0, M_PI/16.0,   M_PI*1/16.0, 0,           M_PI*1/16.0, M_PI/16.0,   M_PI*1/16.0, 0,           M_PI*1/16.0, M_PI/16.0,   M_PI*1/16.0};
const float _notchAdjustStretchLimit = 0.3;
float _aNotchAngles[_noOfNotches] =                       {0,           M_PI/8.0,    M_PI*2/8.0,  M_PI*3/8.0,  M_PI*4/8.0,  M_PI*5/8.0,  M_PI*6/8.0,  M_PI*7/8.0,  M_PI*8/8.0,  M_PI*9/8.0,  M_PI*10/8.0, M_PI*11/8.0, M_PI*12/8.0, M_PI*13/8.0, M_PI*14/8.0, M_PI*15/8.0};
float _measuredNotchAngles[_noOfNotches];
const int _notchStatusDefaults[_noOfNotches] =            {3,           1,           2,           1,           3,           1,           2,           1,           3,           1,           2,           1,           3,           1,           2,           1};
int _aNotchStatus[_noOfNotches] =                         {3,           1,           2,           1,           3,           1,           2,           1,           3,           1,           2,           1,           3,           1,           2,           1};
int _cNotchStatus[_noOfNotches] =                         {3,           1,           2,           1,           3,           1,           2,           1,           3,           1,           2,           1,           3,           1,           2,           1};
float _cNotchAngles[_noOfNotches];
//                                                         up right     up left      down left    down right   notch 1      notch 2      notch 3      notch 4      notch 5      notch 6      notch 7      notch 8
const int _notchAdjOrder[_noOfAdjNotches] =               {2,           6,           10,          14,          1,           3,           5,           7,           9,           11,          13,          15};
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

Bounce bounceDr = Bounce();
Bounce bounceDu = Bounce();
Bounce bounceDl = Bounce();
Bounce bounceDd = Bounce();

ADC *adc = new ADC();


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
//simple low pass filter state variable for c-stick
float _cXPos;
float _cYPos;

#ifdef TEENSY3_2
const int _cmdLengthShort = 5; //number or serial bytes (2 bits per byte) in a short command - 8 bits + stopbit = 5 bytes
const int _cmdLengthLong = 13; //number or serial bytes (2 bits per byte) in a long command - 24 bits + stopbit = 13 bytes
const int _probeLength = 12; //number or serial bytes (2 bits per byte) in a probe response not including the stop bit - 24 bits = 12 bytes
const int _originLength = 40; //number or serial bytes (2 bits per byte) in a origin response not including the stop bit - 80 bits = 40 bytes
const int _pollLength = 32; //number or serial bytes (2 bits per byte) in a origin response not including the stop bit - 64 bits = 32 bytes

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
volatile int _writeQueue = 0;

const char _probeResponse[_probeLength] = {
    0x08,0x08,0x0F,0xE8,
    0x08,0x08,0x08,0x08,
    0x08,0x08,0x08,0xEF};
volatile char _originResponse[_originLength] = {
    0x08,0x08,0x08,0x08,
    0x0F,0x08,0x08,0x08,
    0xE8,0xEF,0xEF,0xEF,
    0xE8,0xEF,0xEF,0xEF,
    0xE8,0xEF,0xEF,0xEF,
    0xE8,0xEF,0xEF,0xEF,
    0x08,0xEF,0xEF,0x08,
    0x08,0xEF,0xEF,0x08,
	0x08,0x08,0x08,0x08,
    0x08,0x08,0x08,0x08};
volatile char _commResponse[_originLength] = {
    0x08,0x08,0x08,0x08,
    0x0F,0x08,0x08,0x08,
    0xE8,0xEF,0xEF,0xEF,
    0xE8,0xEF,0xEF,0xEF,
    0xE8,0xEF,0xEF,0xEF,
    0xE8,0xEF,0xEF,0xEF,
    0x08,0xEF,0xEF,0x08,
    0x08,0xEF,0xEF,0x08,
	0x08,0x08,0x08,0x08,
    0x08,0x08,0x08,0x08};


volatile char _bitCount = 0;
volatile int _commStatus = 0;
const int _commIdle = 0;
const int _commRead = 1;
const int _commPoll = 2;
const int _commWrite = 3;
#endif // TEENSY3_2

#ifdef TEENSY4_0
////Serial settings
bool _writing = false;
bool _waiting = false;
int _bitQueue = 8;
int _waitQueue = 0;
int _writeQueue = 0;
uint8_t _cmdByte = 0;
const int _fastBaud = 2500000;
const int _slowBaud = 2000000;
const int _probeLength = 24;
const int _originLength = 80;
const int _pollLength = 64;
static char _serialBuffer[128];
static char _writeBuffer[128];
int _errorCount = 0;
int _reportCount = 0;

const char _probeResponse[_probeLength] = {
    0,0,0,0, 1,0,0,1,
    0,0,0,0, 0,0,0,0,
    0,0,0,0, 0,0,1,1};
volatile char _originResponse[_originLength] = {
    0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,
    0,1,1,1,1,1,1,1,
    0,1,1,1,1,1,1,1,
    0,1,1,1,1,1,1,1,
    0,1,1,1,1,1,1,1,
    0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0};
volatile char _commResponse[_originLength] = {
    0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,
    0,1,1,1,1,1,1,1,
    0,1,1,1,1,1,1,1,
    0,1,1,1,1,1,1,1,
    0,1,1,1,1,1,1,1,
    0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0};
#endif // TEENSY4_0

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

	const int numberOfNaN = readEEPROM();
	Serial.print("Number of NaN in EEPROM: ");
	Serial.println(numberOfNaN);
	if(numberOfNaN > 3){//by default it seems 4 end up NaN on Teensy 4
		resetDefaults(true);//do reset sticks
		readEEPROM();
	}

	//set some of the unused values in the message response
	_btn.errS = 0;
	_btn.errL = 0;
	_btn.orig = 0;
	_btn.high = 1;

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
    _cXPos = 0;
    _cYPos = 0;

	_lastMicros = micros();

    setPinModes();

    ADCSetup(adc, _ADCScale, _ADCScaleFactor);

	//measure the trigger values
	initializeButtons(_btn,trigL,trigR);
	//set the origin response before the sticks have been touched
	//it will never be changed again after this
	setCommResponse(_originResponse, _btn);

//set upt communication interrupts, serial, and timers
#ifdef TEENSY4_0
    Serial2.addMemoryForRead(_serialBuffer,128);
	attachInterrupt(_pinInt, commInt, RISING);
#ifdef HALFDUPLEX
	Serial2.addMemoryForWrite(_writeBuffer, 128);
	Serial2.begin(_slowBaud,SERIAL_HALF_DUPLEX);
	//Serial2.setTX(8,true);
	timer1.begin(resetSerial);
#endif // HALFDUPLEX
#endif // TEENSY4_0
#ifndef HALFDUPLEX
	Serial2.begin(_slowBaud);
#endif // HALFDUPLEX

#ifdef TEENSY3_2
#ifdef HALFDUPLEX
	Serial2.begin(_slowBaud,SERIAL_HALF_DUPLEX);
#endif // HALFDUPLEX
	timer1.begin(communicate);
	//timer2.begin(checkCmd);
	//timer3.begin(writePole);
	digitalWriteFast(12,HIGH);
	//ARM_DEMCR |= ARM_DEMCR_TRCENA;
	//ARM_DWT_CTRL |= ARM_DWT_CTRL_CYCCNTENA;
	attachInterrupt(_pinInt, bitCounter, FALLING);
	NVIC_SET_PRIORITY(IRQ_PORTC, 0);
#endif // TEENSY3_2
}

void loop() {
	//check if we should be reporting values yet
	if((_btn.B || _autoInit) && !_running){
		Serial.println("Starting to report values");
		_running=true;
	}

	//read the controllers buttons
	readButtons(_btn, _hardware);

	//check to see if we are calibrating
	if(_currentCalStep >= 0){
		if(_calAStick){
			if(_currentCalStep >= _noOfCalibrationPoints){//adjust notch angles
				adjustNotch(_currentCalStep, _dT, _hardware.Y, _hardware.X, _btn.B, true, _measuredNotchAngles, _aNotchAngles, _aNotchStatus);
				if(_hardware.Y || _hardware.X || (_btn.B)){//only run this if the notch was adjusted
					//clean full cal points again, feeding updated angles in
					cleanCalPoints(_tempCalPointsX, _tempCalPointsY, _aNotchAngles, _cleanedPointsX, _cleanedPointsY, _notchPointsX, _notchPointsY, _aNotchStatus);
					//linearize again
					linearizeCal(_cleanedPointsX, _cleanedPointsY, _cleanedPointsX, _cleanedPointsY, _aFitCoeffsX, _aFitCoeffsY);
					//notchCalibrate again to update the affine transform
					notchCalibrate(_cleanedPointsX, _cleanedPointsY, _notchPointsX, _notchPointsY, _noOfNotches, _aAffineCoeffs, _aBoundaryAngles);
				}
			}else{//just show desired stick position
				displayNotch(_currentCalStep, true, _notchAngleDefaults);
			}
			readSticks(true,false, _btn, _hardware);
		}
		else{
			if(_currentCalStep >= _noOfCalibrationPoints){//adjust notch angles
				adjustNotch(_currentCalStep, _dT, _hardware.Y, _hardware.X, _btn.B, false, _measuredNotchAngles, _cNotchAngles, _cNotchStatus);
				if(_hardware.Y || _hardware.X || (_btn.B)){//only run this if the notch was adjusted
					//clean full cal points again, feeding updated angles in
					cleanCalPoints(_tempCalPointsX, _tempCalPointsY, _cNotchAngles, _cleanedPointsX, _cleanedPointsY, _notchPointsX, _notchPointsY, _cNotchStatus);
					//linearize again
					linearizeCal(_cleanedPointsX, _cleanedPointsY, _cleanedPointsX, _cleanedPointsY, _cFitCoeffsX, _cFitCoeffsY);
					//notchCalibrate again to update the affine transform
					notchCalibrate(_cleanedPointsX, _cleanedPointsY, _notchPointsX, _notchPointsY, _noOfNotches, _cAffineCoeffs, _cBoundaryAngles);
				}
			}else{//just show desired stick position
				displayNotch(_currentCalStep, false, _notchAngleDefaults);
			}
			readSticks(false,true, _btn, _hardware);
		}
	}
	else if(_running){
		//if not calibrating read the sticks normally
		readSticks(true,true, _btn, _hardware);
	}
}

#ifdef TEENSY4_0
#ifndef HALFDUPLEX
//commInt() will be called on every rising edge of a pulse that we receive
//we will check if we have the expected amount of serial data yet, if we do we will do something with it, if we don't we will do nothing and wait for the next rising edge to check again
void commInt() {
	//check to see if we have the expected amount of data yet
	if(Serial2.available() >= _bitQueue){
		//check to see if we have been writing data, if have then we need to clear it and set the serial port back to low speed to be ready to receive the next command
		if(_writing){
			//Set pin 13 (LED) low for debugging, if it flickers it means the teensy got stuck here somewhere
			digitalWriteFast(_pinLED,LOW);
			//wait for the stop bit to be read

			while(Serial2.available() <= _bitQueue){}
			//check to see if we just reset reportCount to 0, if we have then we will report the data we just sent over to the PC over serial
			if(_reportCount == 0){
				char myBuffer[128];
				for(int i = 0; i < _bitQueue+1; i++){
					myBuffer[i] = (Serial2.read() > 0b11110000)+48;
				}
				//Serial.print("Sent: ");
				//Serial.write(myBuffer,_bitQueue+1);
				//Serial.println();
			}

			//flush and clear the any remaining data just to be sure
			Serial2.flush();
			Serial2.clear();

			//turn the writing flag off, set the serial port to low speed, and set our expected bit queue to 8 to be ready to receive our next command
			_writing = false;
			Serial2.begin(_slowBaud);
			_bitQueue = 8;
		}
		//if we are not writing, check to see if we were waiting for a poll command to finish
		//if we are, we need to clear the data and send our poll response
		else if(_waiting){
			digitalWriteFast(_pinLED,LOW);
			//wait for the stop bit to be received
			while(Serial2.available() <= _bitQueue){}
#ifdef ENABLE_LED
			digitalWriteFast(_pinLED,HIGH);
#endif //ENABLE_LED
			//check to see if we just reset reportCount to 0, if we have then we will report the remainder of the poll response to the PC over serial
			if(_reportCount == 0){
				Serial.print("Poll: ");
				char myBuffer[128];
				for(int i = 0; i < _bitQueue+1; i++){
					myBuffer[i] = (Serial2.read() > 0b11110000)+48;
				}
				//Serial.write(myBuffer,_bitQueue+1);
				//Serial.println();
			}

			//clear any remaining data
			Serial2.clear();

			//clear any remaining data, set the waiting flag to false, and set the serial port to high speed to be ready to send our poll response
			Serial2.clear();
			_waiting = false;
			setFastBaud();

			//set the writing flag to true, set our expected bit queue to the poll response length -1 (to account for the stop bit)
			_writing = true;
			_bitQueue = _pollLength/2;

			//write the poll response
			for(int i = 0; i<_pollLength; i += 2){
				if(_commResponse[i] != 0 && _commResponse[i+1] != 0){
					//short low period = 1
					//long low period = 0
					Serial2.write(0xEF);
				} else if (_commResponse[i] == 0 && _commResponse[i+1] != 0){
					Serial2.write(0xE8);
				} else if (_commResponse[i] != 0 && _commResponse[i+1] == 0){
					Serial2.write(0x0F);
				} else if (_commResponse[i] == 0 && _commResponse[i+1] == 0){
					Serial2.write(0x08);
				}
			}
			//write stop bit to indicate end of response
			Serial2.write(0xFF);
		}
		else{
			//We are not writing a response or waiting for a poll response to finish, so we must have received the start of a new command
			//Set pin 13 (LED) low for debugging, if it flickers it means the teensy got stuck here somewhere
			digitalWriteFast(_pinLED,LOW);

			//increment the report count, will be used to only send a report every 64 commands to not overload the PC serial connection
			_reportCount++;
			if(_reportCount > 64){
				_reportCount = 0;
			}

			//clear the command byte of previous data
			_cmdByte = 0;

			//write the new data from the serial buffer into the command byte
			for(int i = 0; i<8; i++){
				_cmdByte = (_cmdByte<<1) | (Serial2.read() > 0b11110000);

			}

			//if we just reset reportCount, report the command we received and the number of strange commands we've seen so far over serial
			//if(_reportCount==0){
				//Serial.print("Received: ");
				//Serial.println(_cmdByte,BIN);
				//Serial.print("Error Count:");
				//Serial.println(_errorCount);
			//}

			//if the command byte is all 0s it is probe command, we will send a probe response
			if(_cmdByte == 0b00000000){
				//wait for the stop bit to be received and clear it
				while(!Serial2.available()){}
				Serial2.clear();

				//switch the hardware serial to high speed for sending the response, set the _writing flag to true, and set the expected bit queue length to the probe response length minus 1 (to account for the stop bit)
				setFastBaud();
				_writing = true;
				_bitQueue = _probeLength/2;

				//write the probe response
				for(int i = 0; i<_probeLength; i += 2){
					if(_probeResponse[i] != 0 && _probeResponse[i+1] != 0){
						//short low period = 1
						//long low period = 0
						Serial2.write(0xEF);
					} else if (_probeResponse[i] == 0 && _probeResponse[i+1] != 0){
						Serial2.write(0xE8);
					} else if (_probeResponse[i] != 0 && _probeResponse[i+1] == 0){
						Serial2.write(0x0F);
					} else if (_probeResponse[i] == 0 && _probeResponse[i+1] == 0){
						Serial2.write(0x08);
					}
				}
				//write stop bit to indicate end of response
				Serial2.write(0xFF);
			}
			//if the command byte is 01000001 it is an origin command, we will send an origin response
			else if(_cmdByte == 0b01000001){
				//wait for the stop bit to be received and clear it
				while(!Serial2.available()){}
				Serial2.clear();

				//switch the hardware serial to high speed for sending the response, set the _writing flag to true, and set the expected bit queue length to the origin response length minus 1 (to account for the stop bit)
				setFastBaud();
				_writing = true;
				_bitQueue = _originLength/2;

				//write the origin response
				for(int i = 0; i<_originLength; i += 2){
					if(_originResponse[i] != 0 && _originResponse[i+1] != 0){
						//short low period = 1
						//long low period = 0
						Serial2.write(0xEF);
					} else if (_originResponse[i] == 0 && _originResponse[i+1] != 0){
						Serial2.write(0xE8);
					} else if (_originResponse[i] != 0 && _originResponse[i+1] == 0){
						Serial2.write(0x0F);
					} else if (_originResponse[i] == 0 && _originResponse[i+1] == 0){
						Serial2.write(0x08);
					}
				}
				//write stop bit to indicate end of response
				Serial2.write(0xFF);
			}

			//if the command byte is 01000000 it is an poll command, we need to wait for the poll command to finish then send our poll response
			//to do this we will set our expected bit queue to the remaining length of the poll command, and wait until it is finished
			else if(_cmdByte == 0b01000000){
				_waiting = true;
				_bitQueue = 16;
				setCommResponse(_commResponse, _btn);
			}
			//if we got something else then something went wrong, print the command we got and increase the error count
			else{
				Serial.print("error: ");
				Serial.println(_cmdByte,BIN);
				_errorCount ++;

				//we don't know for sure what state things are in, so clear, flush, and restart the serial port at low speed to be ready to receive a command
				Serial2.clear();
				Serial2.flush();
				Serial2.begin(_slowBaud);
				//set our expected bit queue to 8, which will collect the first byte of any command we receive
				_bitQueue = 8;
			}
		}
	}
	//turn the LED back on to indicate we are not stuck
#ifdef ENABLE_LED
	digitalWriteFast(_pinLED,HIGH);
#endif //ENABLE_LED
}
#else // HALFDUPLEX
//commInt() will be called on every rising edge of a pulse that we receive
//we will check if we have the expected amount of serial data yet, if we do we will do something with it, if we don't we will do nothing and wait for the next rising edge to check again
void commInt() {
	digitalWriteFast(_pinLED,LOW);
	//check to see if we have the expected amount of data yet
	if(Serial2.available() >= _bitQueue){//bitQueue is 8 or 16
		//check to see if we were waiting for a poll command to finish
		//if we are, we need to clear the data and send our poll response
		if(_waiting){
			//wait for the stop bit to be received
			while(Serial2.available() <= _bitQueue){}//bitQueue is 16 if we're _waiting
			//check to see if we just reset reportCount to 0, if we have then we will report the remainder of the poll response to the PC over serial

			//save command byte for later use in setting rumble
			for(int i = 0; i < _bitQueue; i++){
				_cmdByte = (_cmdByte<<1) | (Serial2.read() > 0b11110000);
			}

			//clear any remaining data, set the waiting flag to false, and set the serial port to high speed to be ready to send our poll response
			Serial2.clear();
			_waiting = false;
			_bitQueue = 8;
			setFastBaud();

			//write the poll response
			for(int i = 0; i<_pollLength; i += 2){
				if(_commResponse[i] != 0 && _commResponse[i+1] != 0){
					//short low period = 1
					//long low period = 0
					Serial2.write(0xEF);
				} else if (_commResponse[i] == 0 && _commResponse[i+1] != 0){
					Serial2.write(0xE8);
				} else if (_commResponse[i] != 0 && _commResponse[i+1] == 0){
					Serial2.write(0x0F);
				} else if (_commResponse[i] == 0 && _commResponse[i+1] == 0){
					Serial2.write(0x08);
				}
			}
			//write stop bit to indicate end of response
			Serial2.write(0xFF);

			//start the timer to reset the the serial port when the response has been sent
			timer1.trigger(165);

			//actually write to the rumble pins after beginning the write
#ifdef RUMBLE
			if(_cmdByte & 0b00000001 && _rumble > 0){
				analogWrite(_pinBrake,0);
				analogWrite(_pinRumble, _rumblePower);
			}
			else if(_cmdByte & 0b00000010){
				analogWrite(_pinRumble,0);
				analogWrite(_pinBrake,256);
			}
			else{
				analogWrite(_pinRumble,0);
				analogWrite(_pinBrake,0);
			}
#endif
		}
		else{
			//We are not writing a response or waiting for a poll response to finish, so we must have received the start of a new command
			//increment the report count, will be used to only send a report every 64 commands to not overload the PC serial connection
			_reportCount++;
			if(_reportCount > 64){
				_reportCount = 0;
			}

			//clear the command byte of previous data
			_cmdByte = 0;

			//write the new data from the serial buffer into the command byte
			for(int i = 0; i<8; i++){
				_cmdByte = (_cmdByte<<1) | (Serial2.read() > 0b11110000);

			}

			//if we just reset reportCount, report the command we received and the number of strange commands we've seen so far over serial
			if(_reportCount==0){
				Serial.print("Received: ");
				Serial.println(_cmdByte,BIN);
				Serial.print("Error Count:");
				Serial.println(_errorCount);
			}

			//if the command byte is all 0s it is probe command, we will send a probe response
			if(_cmdByte == 0b00000000){
				//wait for the stop bit to be received and clear it
				while(!Serial2.available()){}
				Serial2.clear();

				//switch the hardware serial to high speed for sending the response, set the _writing flag to true, and set the expected bit queue length to the probe response length minus 1 (to account for the stop bit)
				setFastBaud();
				//Serial2.setTX(8,true);

				//write the probe response
				for(int i = 0; i<_probeLength; i += 2){
					if(_probeResponse[i] != 0 && _probeResponse[i+1] != 0){
						//short low period = 1
						//long low period = 0
						Serial2.write(0xEF);
					} else if (_probeResponse[i] == 0 && _probeResponse[i+1] != 0){
						Serial2.write(0xE8);
					} else if (_probeResponse[i] != 0 && _probeResponse[i+1] == 0){
						Serial2.write(0x0F);
					} else if (_probeResponse[i] == 0 && _probeResponse[i+1] == 0){
						Serial2.write(0x08);
					}
				}
				//write stop bit to indicate end of response
				Serial2.write(0xFF);
				resetSerial();
			}
			//if the command byte is 01000001 it is an origin command, we will send an origin response
			else if(_cmdByte == 0b01000001){
				//wait for the stop bit to be received and clear it
				while(!Serial2.available()){}
				Serial2.clear();

				//switch the hardware serial to high speed for sending the response, set the _writing flag to true, and set the expected bit queue length to the origin response length minus 1 (to account for the stop bit)
				setFastBaud();

				//write the origin response
				for(int i = 0; i<_originLength; i += 2){
					if(_originResponse[i] != 0 && _originResponse[i+1] != 0){
						//short low period = 1
						//long low period = 0
						Serial2.write(0xEF);
					} else if (_originResponse[i] == 0 && _originResponse[i+1] != 0){
						Serial2.write(0xE8);
					} else if (_originResponse[i] != 0 && _originResponse[i+1] == 0){
						Serial2.write(0x0F);
					} else if (_originResponse[i] == 0 && _originResponse[i+1] == 0){
						Serial2.write(0x08);
					}
				}
				//write stop bit to indicate end of response
				Serial2.write(0xFF);

				resetSerial();
			}
			//if the command byte is 01000000 it is an poll command, we need to wait for the poll command to finish then send our poll response
			//to do this we will set our expected bit queue to the remaining length of the poll command, and wait until it is finished
			else if(_cmdByte == 0b01000000){
				//digitalWriteFast(_pinLED,LOW);
				_waiting = true;
				_bitQueue = 16;
				setCommResponse(_commResponse, _btn);
			}
			//if we got something else then something went wrong, print the command we got and increase the error count
			else{
				//digitalWriteFast(_pinLED,LOW);
				Serial.print("error: ");
				Serial.println(_cmdByte,BIN);
				_errorCount ++;
				_waiting = false;

				//we don't know for sure what state things are in, so clear, flush, and restart the serial port at low speed to be ready to receive a command
				resetSerial();
				//set our expected bit queue to 8, which will collect the first byte of any command we receive
				_bitQueue = 8;
				//wait a bit to make sure whatever command didn't get read properly is finished
				delayMicroseconds(200);

			}
		}
	}
	//turn the LED back on to indicate we are not stuck
#ifdef ENABLE_LED
	digitalWriteFast(_pinLED,HIGH);
#endif //ENABLE_LED
}
void resetSerial(){
#ifdef ENABLE_LED
	digitalWriteFast(_pinLED,!digitalReadFast(_pinLED));
#endif //ENABLE_LED
	Serial2.clear();
	Serial2.flush();
	Serial2.begin(_slowBaud,SERIAL_HALF_DUPLEX);
	digitalWriteFast(_pinLED,LOW);
}
#endif // HALFDUPLEX
//We were using Serial2.begin() to change baudrate, but that took *waaay* too long.
void setFastBaud(){
	//By using 2:1 bit ratios, we were locking ourselves into a bad baudrate on teensy 4.
	//The nearest lower one to the ideal 250 was 240 kHz, and the nearest higher was 266.
	//oversample ratio * divisor = 24 MHz / baudrate (2.5 MHz) = 9.6,  10*1
	//266 kHz was occasionally too fast for some consoles and adapters.
	//240 kHz was occasionally too slow for Smashscope 2kHz polling until I made this function.
	//I don't know why, but occasionally Smashscope polls a lot faster than every 500 microseconds.
	//Most of the time it's a little slower than 2kHz but then it catches up or something?
	//const int osr = 10;
	//const int div = 1;

	//However, I realized that instead of using 2:1 bit ratios, I could do something similar to on the Teensy 3 code
	//This lets us use a slower uart baudrate and get closer to the ideal of 1.25 MHz, within about 1%.
	//The resulting measured bitrate is 252 KHz.

	//Gonna try 1.25 MHz instead of 2.4
	const int osr = 19;
	const int div = 1;
	IMXRT_LPUART4.BAUD = LPUART_BAUD_OSR(osr-1) | LPUART_BAUD_SBR(div);
}
#endif // TEENSY4_0
int readEEPROM(){
	int numberOfNaN = 0;

	//get the jump setting
	EEPROM.get(_eepromJump, _jumpConfig);
	if(_jumpConfig < _jumpConfigMin){
		_jumpConfig = DEFAULT;
		numberOfNaN++;
	}
	if(_jumpConfig > _jumpConfigMax){
		_jumpConfig = DEFAULT;
		numberOfNaN++;
	}
	setJump(_jumpConfig);

	//get the L setting
	EEPROM.get(_eepromLToggle, _lConfig);
	if(_lConfig < _triggerConfigMin) {
		_lConfig = _triggerDefault;
		numberOfNaN++;
	}
	if(_lConfig > _triggerConfigMax) {
		_lConfig = _triggerDefault;
		numberOfNaN++;
	}

	//get the R setting
	EEPROM.get(_eepromRToggle, _rConfig);
	if(_rConfig < _triggerConfigMin) {
		_rConfig = _triggerDefault;
		numberOfNaN++;
	}
	if(_rConfig > _triggerConfigMax) {
		_rConfig = _triggerDefault;
		numberOfNaN++;
	}

	//get the C-stick X offset
	EEPROM.get(_eepromcXOffset, _cXOffset);
	if(_cXOffset > _cMax) {
		_cXOffset = 0;
		numberOfNaN++;
	} else if(_cXOffset < _cMin) {
		_cXOffset = 0;
		numberOfNaN++;
	}

	//get the C-stick Y offset
	EEPROM.get(_eepromcYOffset, _cYOffset);
	if(_cYOffset > _cMax) {
		_cYOffset = 0;
		numberOfNaN++;
	} else if(_cYOffset < _cMin) {
		_cYOffset = 0;
		numberOfNaN++;
	}

	//get the x-axis snapback correction
	EEPROM.get(_eepromxSnapback, _xSnapback);
	Serial.print("the xSnapback value from eeprom is:");
	Serial.println(_xSnapback);
	if(_xSnapback < _snapbackMin) {
		_xSnapback = _snapbackMin;
		numberOfNaN++;
	} else if (_xSnapback > _snapbackMax) {
		_xSnapback = _snapbackMax;
		numberOfNaN++;
	}
	_gains.xVelDamp = velDampFromSnapback(_xSnapback);
	Serial.print("the xVelDamp value from eeprom is:");
	Serial.println(_gains.xVelDamp);

	//get the y-ayis snapback correction
	EEPROM.get(_eepromySnapback, _ySnapback);
	Serial.print("the ySnapback value from eeprom is:");
	Serial.println(_ySnapback);
	if(_ySnapback < _snapbackMin) {
		_ySnapback = _snapbackMin;
		numberOfNaN++;
	} else if (_ySnapback > _snapbackMax) {
		_ySnapback = _snapbackMax;
		numberOfNaN++;
	}
	_gains.yVelDamp = velDampFromSnapback(_ySnapback);
	Serial.print("the yVelDamp value from eeprom is:");
	Serial.println(_gains.yVelDamp);

	//get the x-axis smoothing value
	EEPROM.get(_eepromxSmoothing, _gains.xSmoothing);
	Serial.print("the xSmoothing value from eeprom is:");
	Serial.println(_gains.xSmoothing);
	if(std::isnan(_gains.xSmoothing)){
		_gains.xSmoothing = _smoothingMin;
		Serial.print("the xSmoothing value was adjusted to:");
		Serial.println(_gains.xSmoothing);
		numberOfNaN++;
	}
	if(_gains.xSmoothing > _smoothingMax) {
		_gains.xSmoothing = _smoothingMax;
	} else if(_gains.xSmoothing < _smoothingMin) {
		_gains.xSmoothing = _smoothingMin;
	}

	//get the y-axis smoothing value
	EEPROM.get(_eepromySmoothing, _gains.ySmoothing);
	Serial.print("the ySmoothing value from eeprom is:");
	Serial.println(_gains.ySmoothing);
	if(std::isnan(_gains.ySmoothing)){
		_gains.ySmoothing = _smoothingMin;
		Serial.print("the ySmoothing value was adjusted to:");
		Serial.println(_gains.ySmoothing);
		numberOfNaN++;
	}
	if(_gains.ySmoothing > _smoothingMax) {
		_gains.ySmoothing = _smoothingMax;
	} else if(_gains.ySmoothing < _smoothingMin) {
		_gains.ySmoothing = _smoothingMin;
	}

	//get the c-stick x-axis smoothing value
	EEPROM.get(_eepromCxSmoothing, _gains.cXSmoothing);
	Serial.print("the cXSmoothing value from eeprom is:");
	Serial.println(_gains.cXSmoothing);
	if(std::isnan(_gains.cXSmoothing)){
		_gains.cXSmoothing = _smoothingMin;
		Serial.print("the cXSmoothing value was adjusted to:");
		Serial.println(_gains.cXSmoothing);
		numberOfNaN++;
	}
	if(_gains.cXSmoothing > _smoothingMax) {
		_gains.cXSmoothing = _smoothingMax;
	} else if(_gains.cXSmoothing < _smoothingMin) {
		_gains.cXSmoothing = _smoothingMin;
	}

	//get the c-stick y-axis smoothing value
	EEPROM.get(_eepromCySmoothing, _gains.cYSmoothing);
	Serial.print("the cYSmoothing value from eeprom is:");
	Serial.println(_gains.cYSmoothing);
	if(std::isnan(_gains.cYSmoothing)){
		_gains.cYSmoothing = _smoothingMin;
		Serial.print("the cYSmoothing value was adjusted to:");
		Serial.println(_gains.cYSmoothing);
		numberOfNaN++;
	}
	if(_gains.cYSmoothing > _smoothingMax) {
		_gains.cYSmoothing = _smoothingMax;
	} else if(_gains.cYSmoothing < _smoothingMin) {
		_gains.cYSmoothing = _smoothingMin;
	}

	//recompute the intermediate gains used directly by the kalman filter
	recomputeGains();

	//get the L-trigger Offset value
	EEPROM.get(_eepromLOffset, _LTriggerOffset);
	if(_LTriggerOffset > _triggerMax) {
		_LTriggerOffset = _triggerMax;
		numberOfNaN++;
	} else if(_LTriggerOffset < _triggerMin) {
		_LTriggerOffset = _triggerMin;
		numberOfNaN++;
	}

	//get the R-trigger Offset value
	EEPROM.get(_eepromROffset, _RTriggerOffset);
	if(_RTriggerOffset > _triggerMax) {
		_RTriggerOffset = _triggerMax;
		numberOfNaN++;
	} else if(_RTriggerOffset < _triggerMin) {
		_RTriggerOffset = _triggerMin;
		numberOfNaN++;
	}

	//Get the rumble value
	EEPROM.get(_eepromRumble, _rumble);
	Serial.print("Rumble value before fixing: ");
	Serial.println(_rumble);
	if(std::isnan(_rumble)) {
		_rumble = _rumbleDefault;
		numberOfNaN++;
	}
	if(_rumble < _rumbleMin) {
		_rumble = _rumbleMin;
	}
	if(_rumble > _rumbleMax) {
		_rumble = _rumbleMax;
	}
	_rumblePower = calcRumblePower(_rumble);
	Serial.print("Rumble value: ");
	Serial.println(_rumble);
	Serial.print("Rumble power: ");
	Serial.println(_rumblePower);

	//Get the autoinit value
	EEPROM.get(_eepromAutoInit, _autoInit);
	if(_autoInit < 0) {
		_autoInit = 0;
		numberOfNaN++;
	}
	if(_autoInit > 1) {
		_autoInit = 0;
		numberOfNaN++;
	}
	Serial.print("Auto init: ");
	Serial.println(_autoInit);

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

void resetDefaults(bool resetSticks){
	Serial.println("RESETTING ALL DEFAULTS");

	_jumpConfig = DEFAULT;
	setJump(_jumpConfig);
	EEPROM.put(_eepromJump,_jumpConfig);

	_lConfig = _triggerDefault;
	_rConfig = _triggerDefault;
	EEPROM.put(_eepromLToggle, _lConfig);
	EEPROM.put(_eepromRToggle, _rConfig);

	_cXOffset = 0;
	_cYOffset = 0;
	EEPROM.put(_eepromcXOffset, _cXOffset);
	EEPROM.put(_eepromcYOffset, _cYOffset);

	_xSnapback = _snapbackDefault;
	EEPROM.put(_eepromxSnapback,_xSnapback);
	_gains.xVelDamp = velDampFromSnapback(_xSnapback);
	_ySnapback = _snapbackDefault;
	EEPROM.put(_eepromySnapback,_ySnapback);
	_gains.yVelDamp = velDampFromSnapback(_ySnapback);

	_gains.xSmoothing = _smoothingMin;
	EEPROM.put(_eepromxSmoothing, _gains.xSmoothing);
	_gains.ySmoothing = _smoothingMin;
	EEPROM.put(_eepromySmoothing, _gains.ySmoothing);

	_gains.cXSmoothing = _smoothingMin;
	EEPROM.put(_eepromCxSmoothing, _gains.cXSmoothing);
	_gains.cYSmoothing = _smoothingMin;
	EEPROM.put(_eepromCySmoothing, _gains.cYSmoothing);
	//recompute the intermediate gains used directly by the kalman filter
	recomputeGains();

	_LTriggerOffset = _triggerMin;
	_RTriggerOffset = _triggerMin;
	EEPROM.put(_eepromLOffset, _LTriggerOffset);
	EEPROM.put(_eepromROffset, _RTriggerOffset);

	_rumble = _rumbleDefault;
	_rumblePower = calcRumblePower(_rumble);
	EEPROM.put(_eepromRumble, _rumble);

	//always cancel auto init on reset, even if we don't reset the sticks
	_autoInit = 0;
	EEPROM.put(_eepromAutoInit, _autoInit);

	if(resetSticks){
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

	bounceDr.attach(_pinDr);
	bounceDr.interval(1000);
	bounceDu.attach(_pinDu);
	bounceDu.interval(1000);
	bounceDl.attach(_pinDl);
	bounceDl.interval(1000);
	bounceDd.attach(_pinDd);
	bounceDd.interval(1000);
}
void readButtons(Buttons &btn, HardwareButtons &hardware){
	btn.A = !digitalRead(_pinA);
	btn.B = !digitalRead(_pinB);
	btn.X = !digitalRead(_pinXSwappable);
	btn.Y = !digitalRead(_pinYSwappable);
	btn.Z = !digitalRead(_pinZSwappable);
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

	if(hardware.L && hardware.R && btn.A && btn.S) {
		btn.L = (uint8_t) (1);
		btn.R = (uint8_t) (1);
		btn.A = (uint8_t) (1);
		btn.S = (uint8_t) (1);
	} else {
		switch(_lConfig) {
			case 0: //Default Trigger state
				btn.L = hardware.L;
				break;
			case 1: //Digital Only Trigger state
				btn.L = hardware.L;
				break;
			case 2: //Analog Only Trigger state
				btn.L = (uint8_t) 0;
				break;
			case 3: //Trigger Plug Emulation state
				btn.L = hardware.L;
				break;
			case 4: //Digital => Analog Value state
				btn.L = (uint8_t) 0;
				break;
			case 5: //Digital -> Analog Value + Digital state
				btn.L = hardware.L;
				break;
			default:
				btn.L = hardware.L;
		}

		switch(_rConfig) {
			case 0: //Default Trigger state
				btn.R = hardware.R;
				break;
			case 1: //Digital Only Trigger state
				btn.R = hardware.R;
				break;
			case 2: //Analog Only Trigger state
				btn.R = (uint8_t) 0;
				break;
			case 3: //Trigger Plug Emulation state
				btn.R = hardware.R;
				break;
			case 4: //Digital => Analog Value state
				btn.R = (uint8_t) 0;
				break;
			case 5: //Digital -> Analog Value + Digital state
				btn.R = hardware.R;
				break;
			default:
				btn.R = hardware.R;
		}
	}

	bounceDr.update();
	bounceDu.update();
	bounceDl.update();
	bounceDd.update();


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
	if(!_safeMode && (_currentCalStep == -1)) {
		if(btn.A && hardware.X && hardware.Y && btn.S) { //Safe Mode Toggle
			_safeMode = true;
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
			resetDefaults(false);//don't reset sticks
			freezeSticks(2000, btn, hardware);
		} else if (btn.A && btn.B && hardware.Z && btn.Dd) { //Hard Reset
			resetDefaults(true);//do reset sticks
			freezeSticks(2000, btn, hardware);
		} else if (btn.A && btn.B && hardware.L && hardware.R && btn.S) { //Toggle Auto-Initialize
			changeAutoInit();
		} else if (hardware.X && hardware.Y && btn.Du) { //Increase Rumble
#ifdef RUMBLE
			changeRumble(true, btn, hardware);
#else // RUMBLE
			//nothing
			freezeSticks(2000, btn, hardware);
#endif // RUMBLE
		} else if (hardware.X && hardware.Y && btn.Dd) { //Decrease Rumble
#ifdef RUMBLE
			changeRumble(false, btn, hardware);
#else // RUMBLE
			//nothing
			freezeSticks(2000, btn, hardware);
#endif // RUMBLE
		} else if (hardware.X && hardware.Y && btn.B && !btn.A) { //Show current rumble setting
#ifdef RUMBLE
			showRumble(2000, btn, hardware);
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
			adjustSnapback(true, true, true);
		} else if(hardware.L && hardware.X && btn.Dd) { //Decrease Analog X-Axis Snapback Filtering
			adjustSnapback(true, true, false);
		} else if(hardware.L && hardware.Y && btn.Du) { //Increase Analog Y-Axis Snapback Filtering
			adjustSnapback(true, false, true);
		} else if(hardware.L && hardware.Y && btn.Dd) { //Decrease Analog Y-Axis Snapback Filtering
			adjustSnapback(true, false, false);
		} else if(hardware.L && btn.A && btn.Du) { //Increase X-axis Delay
			adjustSmoothing(true, true, true);
		} else if(hardware.L && btn.A && btn.Dd) { //Decrease X-axis Delay
			adjustSmoothing(true, true, false);
		} else if(hardware.L && btn.B && btn.Du) { //Increase Y-axis Delay
			adjustSmoothing(true, false, true);
		} else if(hardware.L && btn.B && btn.Dd) { //Decrease Y-axis Delay
			adjustSmoothing(true, false, false);
		} else if(hardware.L && btn.S && btn.Dd) { //Show Current Analog Settings
			showAstickSettings();
		} else if(hardware.R && hardware.X && btn.Du) { //Increase C-stick X-Axis Snapback Filtering
			adjustCstickSmoothing(true, true, true);
		} else if(hardware.R && hardware.X && btn.Dd) { //Decrease C-stick X-Axis Snapback Filtering
			adjustCstickSmoothing(true, true, false);
		} else if(hardware.R && hardware.Y && btn.Du) { //Increase C-stick Y-Axis Snapback Filtering
			adjustCstickSmoothing(true, false, true);
		} else if(hardware.R && hardware.Y && btn.Dd) { //Decrease C-stick Y-Axis Snapback Filtering
			adjustCstickSmoothing(true, false, false);
		} else if(hardware.R && btn.A && btn.Du) { //Increase C-stick X Offset
			adjustCstickOffset(true, true, true);
		} else if(hardware.R && btn.A && btn.Dd) { //Decrease C-stick X Offset
			adjustCstickOffset(true, true, false);
		} else if(hardware.R && btn.B && btn.Du) { //Increase C-stick Y Offset
			adjustCstickOffset(true, false, true);
		} else if(hardware.R && btn.B && btn.Dd) { //Decrease C-stick Y Offset
			adjustCstickOffset(true, false, false);
		} else if(hardware.R && btn.S && btn.Dd) { //Show Current C-stick SEttings
			showCstickSettings();
		} else if(hardware.L && hardware.Z && btn.S) { //Toggle Analog L
			nextTriggerState(_lConfig, true);
		} else if(hardware.R && hardware.Z && btn.S) { //Toggle Analog R
			nextTriggerState(_rConfig, false);
		} else if(hardware.L && hardware.Z && btn.Du) { //Increase L-Trigger Offset
			adjustTriggerOffset(true, true, true);
		} else if(hardware.L && hardware.Z && btn.Dd) { //Decrease L-trigger Offset
			adjustTriggerOffset(true, true, false);
		} else if(hardware.R && hardware.Z && btn.Du) { //Increase R-trigger Offset
			adjustTriggerOffset(true, false, true);
		} else if(hardware.R && hardware.Z && btn.Dd) { //Decrease R-trigger Offset
			adjustTriggerOffset(true, false, false);
		} else if(hardware.X && hardware.Z && btn.S) { //Swap X and Z
			readJumpConfig(SWAP_XZ);
			freezeSticks(2000, btn, hardware);
		} else if(hardware.Y && hardware.Z && btn.S) { //Swap Y and Z
			readJumpConfig(SWAP_YZ);
			freezeSticks(2000, btn, hardware);
		} else if(btn.A && hardware.X && hardware.Y && hardware.Z) { // Reset X/Y/Z Config
			readJumpConfig(DEFAULT);
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
			_safeMode = false;
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
				collectCalPoints(_calAStick, _currentCalStep,_tempCalPointsX,_tempCalPointsY);
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
				_autoInit = 0;
				EEPROM.put(_eepromAutoInit, _autoInit);
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
				collectCalPoints(_calAStick, _currentCalStep,_tempCalPointsX,_tempCalPointsY);
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
				_autoInit = 0;
				EEPROM.put(_eepromAutoInit, _autoInit);
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
void changeRumble(const bool increase, Buttons &btn, HardwareButtons &hardware) {
	Serial.println("changing rumble");
	if(increase) {
		_rumble += 1;
	} else {
		_rumble -= 1;
	}
	if(_rumble > _rumbleMax) {
		_rumble = _rumbleMax;
	}
	if(_rumble < _rumbleMin) {
		_rumble = _rumbleMin;
	}

	_rumblePower = calcRumblePower(_rumble);
	showRumble(1000, btn, hardware);
}

void showRumble(const int time, Buttons &btn, HardwareButtons &hardware) {
	_btn.Cx = (uint8_t) 127;
	_btn.Cy = (uint8_t) (_rumble + 127.5);
	clearButtons(time, btn, hardware);

	EEPROM.put(_eepromRumble, _rumble);
}

//Make it so you don't need to press B.
//This is only good if the sticks are calibrated, so
// the setting auto-resets whenever you hard reset or recalibrate.
void changeAutoInit() {
	if(_autoInit == 0) {
		_autoInit = 1;
	} else {
		_autoInit = 0;
	}

	//move sticks up-right for on, down-left for off
	_btn.Ax = (uint8_t) (_autoInit*100 - 50 + 127.5);
	_btn.Ay = (uint8_t) (_autoInit*100 - 50 + 127.5);
	_btn.Cx = (uint8_t) (_autoInit*100 - 50 + 127.5);
	_btn.Cy = (uint8_t) (_autoInit*100 - 50 + 127.5);

	clearButtons(2000, _btn, _hardware);

	EEPROM.put(_eepromAutoInit, _autoInit);
}

void adjustSnapback(bool _change, bool _xAxis, bool _increase){
	Serial.println("adjusting snapback filtering");
	if(_xAxis && _increase && _change){
		_xSnapback = min(_xSnapback+1, _snapbackMax);
		Serial.print("x snapback filtering increased to:");
		Serial.println(_xSnapback);
	}
	else if(_xAxis && !_increase && _change){
		_xSnapback = max(_xSnapback-1, _snapbackMin);
		Serial.print("x snapback filtering decreased to:");
		Serial.println(_xSnapback);
	}

	if(!_xAxis && _increase && _change){
		_ySnapback = min(_ySnapback+1, _snapbackMax);
		Serial.print("y snapback filtering increased to:");
		Serial.println(_ySnapback);
	}
	else if(!_xAxis && !_increase && _change){
		_ySnapback = max(_ySnapback-1, _snapbackMin);
		Serial.print("y snapback filtering decreased to:");
		Serial.println(_ySnapback);
	}

	_gains.xVelDamp = velDampFromSnapback(_xSnapback);
	_gains.yVelDamp = velDampFromSnapback(_ySnapback);

    //recompute the intermediate gains used directly by the kalman filter
    recomputeGains();

	_btn.Cx = (uint8_t) (_xSnapback + 127.5);
	_btn.Cy = (uint8_t) (_ySnapback + 127.5);

	clearButtons(2000, _btn, _hardware);

	EEPROM.put(_eepromxSnapback,_xSnapback);
	EEPROM.put(_eepromySnapback,_ySnapback);
}
void adjustSmoothing(bool _change, bool _xAxis, bool _increase) {
	Serial.println("Adjusting Smoothing");
	if (_xAxis && _increase && _change) {
		_gains.xSmoothing = _gains.xSmoothing + 0.1;
		if(_gains.xSmoothing > _smoothingMax) {
			_gains.xSmoothing = _smoothingMax;
		}
		EEPROM.put(_eepromxSmoothing, _gains.xSmoothing);
		Serial.print("X Smoothing increased to:");
		Serial.println(_gains.xSmoothing);
	} else if(_xAxis && !_increase && _change) {
		_gains.xSmoothing = _gains.xSmoothing - 0.1;
		if(_gains.xSmoothing < _smoothingMin) {
			_gains.xSmoothing = _smoothingMin;
		}
		EEPROM.put(_eepromxSmoothing, _gains.xSmoothing);
		Serial.print("X Smoothing decreased to:");
		Serial.println(_gains.xSmoothing);
	} else if(!_xAxis && _increase && _change) {
		_gains.ySmoothing = _gains.ySmoothing + 0.1;
		if (_gains.ySmoothing > _smoothingMax) {
			_gains.ySmoothing = _smoothingMax;
		}
		EEPROM.put(_eepromySmoothing, _gains.ySmoothing);
		Serial.print("Y Smoothing increased to:");
		Serial.println(_gains.ySmoothing);
	} else if(!_xAxis && !_increase && _change) {
		_gains.ySmoothing = _gains.ySmoothing - 0.1;
		if (_gains.ySmoothing < _smoothingMin) {
			_gains.ySmoothing = _smoothingMin;
		}
		EEPROM.put(_eepromySmoothing, _gains.ySmoothing);
		Serial.print("Y Smoothing decreased to:");
		Serial.println(_gains.ySmoothing);
	}

	//recompute the intermediate gains used directly by the kalman filter
	recomputeGains();

	_btn.Cx = (uint8_t) (127.5 + (_gains.xSmoothing * 10));
	_btn.Cy = (uint8_t) (127.5 + (_gains.ySmoothing * 10));

	clearButtons(2000, _btn, _hardware);
}
void showAstickSettings() {
	//Snapback on A-stick
	_btn.Ax = (uint8_t) (_xSnapback + 127.5);
	_btn.Ay = (uint8_t) (_ySnapback + 127.5);

	//Smoothing on C-stick
	_btn.Cx = (uint8_t) (127.5 + (_gains.xSmoothing * 10));
	_btn.Cy = (uint8_t) (127.5 + (_gains.ySmoothing * 10));

	clearButtons(2000, _btn, _hardware);
}
void adjustCstickSmoothing(bool _change, bool _xAxis, bool _increase) {
	Serial.println("Adjusting C-Stick Smoothing");
	if (_xAxis && _increase && _change) {
		_gains.cXSmoothing = _gains.cXSmoothing + 0.1;
		if(_gains.cXSmoothing > _smoothingMax) {
			_gains.cXSmoothing = _smoothingMax;
		}
		EEPROM.put(_eepromCxSmoothing, _gains.cXSmoothing);
		Serial.print("C-Stick X Smoothing increased to:");
		Serial.println(_gains.cXSmoothing);
	} else if(_xAxis && !_increase && _change) {
		_gains.cXSmoothing = _gains.cXSmoothing - 0.1;
		if(_gains.cXSmoothing < _smoothingMin) {
			_gains.cXSmoothing = _smoothingMin;
		}
		EEPROM.put(_eepromCxSmoothing, _gains.cXSmoothing);
		Serial.print("C-Stick X Smoothing decreased to:");
		Serial.println(_gains.cXSmoothing);
	} else if(!_xAxis && _increase && _change) {
		_gains.cYSmoothing = _gains.cYSmoothing + 0.1;
		if (_gains.cYSmoothing > _smoothingMax) {
			_gains.cYSmoothing = _smoothingMax;
		}
		EEPROM.put(_eepromCySmoothing, _gains.cYSmoothing);
		Serial.print("C-Stick Y Smoothing increased to:");
		Serial.println(_gains.cYSmoothing);
	} else if(!_xAxis && !_increase && _change) {
		_gains.cYSmoothing = _gains.cYSmoothing - 0.1;
		if (_gains.cYSmoothing < _smoothingMin) {
			_gains.cYSmoothing = _smoothingMin;
		}
		EEPROM.put(_eepromCySmoothing, _gains.cYSmoothing);
		Serial.print("C-Stick Y Smoothing decreased to:");
		Serial.println(_gains.cYSmoothing);
	}

	//recompute the intermediate gains used directly by the kalman filter
	recomputeGains();

	_btn.Cx = (uint8_t) (127.5 + (_gains.cXSmoothing * 10));
	_btn.Cy = (uint8_t) (127.5 + (_gains.cYSmoothing * 10));

	clearButtons(2000, _btn, _hardware);
}
void adjustCstickOffset(bool _change, bool _xAxis, bool _increase) {
	Serial.println("Adjusting C-stick Offset");
	if(_xAxis && _increase && _change) {
		_cXOffset++;
		if(_cXOffset > _cMax) {
			_cXOffset = _cMax;
		}
		EEPROM.put(_eepromcXOffset, _cXOffset);
		Serial.print("X offset increased to:");
		Serial.println(_cXOffset);
	} else if(_xAxis && !_increase && _change) {
		_cXOffset--;
		if(_cXOffset < _cMin) {
			_cXOffset = _cMin;
		}
		EEPROM.put(_eepromcXOffset, _cXOffset);
		Serial.print("X offset decreased to:");
		Serial.println(_cXOffset);
	} else if(!_xAxis && _increase && _change) {
		_cYOffset++;
		if(_cYOffset > _cMax) {
			_cYOffset = _cMax;
		}
		EEPROM.put(_eepromcYOffset, _cYOffset);
		Serial.print("Y offset increased to:");
		Serial.println(_cYOffset);
	} else if(!_xAxis && !_increase && _change) {
		_cYOffset--;
		if(_cYOffset < _cMin) {
			_cYOffset = _cMin;
		}
		EEPROM.put(_eepromcYOffset, _cYOffset);
		Serial.print("Y offset decreased to:");
		Serial.println(_cYOffset);
	}

	_btn.Cx = (uint8_t) (127.5 + _cXOffset);
	_btn.Cy = (uint8_t) (127.5 + _cYOffset);

	clearButtons(2000, _btn, _hardware);
}
void showCstickSettings() {
	//Snapback/smoothing on A-stick
	_btn.Ax = (uint8_t) (127.5 + (_gains.cXSmoothing * 10));
	_btn.Ay = (uint8_t) (127.5 + (_gains.cYSmoothing * 10));

	//Smoothing on C-stick
	_btn.Cx = (uint8_t) (127.5 + _cXOffset);
	_btn.Cy = (uint8_t) (127.5 + _cYOffset);

	clearButtons(2000, _btn, _hardware);
}
void adjustTriggerOffset(bool _change, bool _lTrigger, bool _increase) {
	if(_lTrigger && _increase && _change) {
		_LTriggerOffset++;
		if(_LTriggerOffset > _triggerMax) {
			_LTriggerOffset = _triggerMax;
		}
	} else if(_lTrigger && !_increase && _change) {
		_LTriggerOffset--;
		if(_LTriggerOffset < _triggerMin) {
			_LTriggerOffset = _triggerMin;
		}
	} else if(!_lTrigger && _increase && _change) {
		_RTriggerOffset++;
		if(_RTriggerOffset > _triggerMax) {
			_RTriggerOffset = _triggerMax;
		}
	} else if(!_lTrigger && !_increase && _change) {
		_RTriggerOffset--;
		if(_RTriggerOffset < _triggerMin) {
			_RTriggerOffset = _triggerMin;
		}
	}

	EEPROM.put(_eepromLOffset, _LTriggerOffset);
	EEPROM.put(_eepromROffset, _RTriggerOffset);

	if(_LTriggerOffset > 99) {
		_btn.Ax = (uint8_t) (127.5 + 100);
		_btn.Cx = (uint8_t) (127.5 + _LTriggerOffset-100);
	} else {
		_btn.Cx = (uint8_t) (127.5 + _LTriggerOffset);
	}
	if(_RTriggerOffset > 99) {
		_btn.Ay = (uint8_t) (127.5 + 100);
		_btn.Cy = (uint8_t) (127.5 + _RTriggerOffset-100);
	} else {
		_btn.Cy = (uint8_t) (127.5 + _RTriggerOffset);
	}

	clearButtons(250, _btn, _hardware);
}
void readJumpConfig(JumpConfig jumpConfig){
	Serial.print("setting jump to: ");
	if (_jumpConfig == jumpConfig) {
		_jumpConfig = DEFAULT;
		Serial.println("normal again");
	} else {
		_jumpConfig = jumpConfig;
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
	EEPROM.put(_eepromJump,_jumpConfig);
	setJump(_jumpConfig);
}
void setJump(int jumpConfig){
	switch(jumpConfig){
			case SWAP_XZ:
				_pinZSwappable = _pinX;
				_pinXSwappable = _pinZ;
				_pinYSwappable = _pinY;
				break;
			case SWAP_YZ:
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
void nextTriggerState(int _currentConfig, bool _lTrigger) {
	if(_lTrigger) {
		if(_currentConfig >= _triggerConfigMax) {
			_lConfig = 0;
		} else {
			_lConfig = _currentConfig + 1;
		}
	} else {
		if(_currentConfig >= _triggerConfigMax) {
			_rConfig = 0;
		} else {
			_rConfig = _currentConfig + 1;
		}
	}
	EEPROM.put(_eepromLToggle, _lConfig);
	EEPROM.put(_eepromRToggle, _rConfig);

	//We want to one-index the modes for the users, so we add 1 here
	_btn.Ay = (uint8_t) (127.5);
	_btn.Ax = (uint8_t) (127.5 + _lConfig + 1);
	_btn.Cy = (uint8_t) (127.5);
	_btn.Cx = (uint8_t) (127.5 + _rConfig + 1);

	clearButtons(2000, _btn, _hardware);
}
void initializeButtons(Buttons &btn,int &startUpLa, int &startUpRa){
	//set the analog stick values to the chosen center value that will be reported to the console on startup
	btn.Ax = _analogCenter;
	btn.Ay = _analogCenter;
	btn.Cx = _analogCenter;
	btn.Cy = _analogCenter;

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
void readSticks(int readA, int readC, Buttons &btn, HardwareButtons &hardware){
#ifdef USEADCSCALE
	_ADCScale = _ADCScale*0.999 + _ADCScaleFactor/adc->adc1->analogRead(ADC_INTERNAL_SOURCE::VREF_OUT);
#endif
	// otherwise _ADCScale is 1

	//read the L and R sliders
	switch(_lConfig) {
		case 0: //Default Trigger state
			btn.La = adc->adc0->analogRead(_pinLa)>>4;
			break;
		case 1: //Digital Only Trigger state
			btn.La = (uint8_t) 0;
			break;
		case 2: //Analog Only Trigger state
			btn.La = adc->adc0->analogRead(_pinLa)>>4;
			break;
		case 3: //Trigger Plug Emulation state
			btn.La = adc->adc0->analogRead(_pinLa)>>4;
			if (btn.La > (((uint8_t) (_LTriggerOffset)) + trigL)) {
				btn.La = (((uint8_t) (_LTriggerOffset)) + trigL);
			}
			break;
		case 4: //Digital => Analog Value state
			if(hardware.L) {
				btn.La = min(((uint8_t) (_LTriggerOffset)) + trigL, 255);
			} else {
				btn.La = (uint8_t) 0;
			}
			break;
		case 5: //Digital => Analog Value + Digital state
			if(hardware.L) {
				btn.La = min(((uint8_t) (_LTriggerOffset)) + trigL, 255);
			} else {
				btn.La = (uint8_t) 0;
			}
			break;
		default:
			btn.La = adc->adc0->analogRead(_pinLa)>>4;
	}

	switch(_rConfig) {
		case 0: //Default Trigger state
			btn.Ra = adc->adc0->analogRead(_pinRa)>>4;
			break;
		case 1: //Digital Only Trigger state
			btn.Ra = (uint8_t) 0;
			break;
		case 2: //Analog Only Trigger state
			btn.Ra = adc->adc0->analogRead(_pinRa)>>4;
			break;
		case 3: //Trigger Plug Emulation state
			btn.Ra = adc->adc0->analogRead(_pinRa)>>4;
			if (btn.Ra > (((uint8_t) (_RTriggerOffset)) + trigR)) {
				btn.Ra = (((uint8_t) (_RTriggerOffset)) + trigR);
			}
			break;
		case 4: //Digital => Analog Value state
			if(hardware.R) {
				btn.Ra = min(((uint8_t) (_RTriggerOffset)) + trigR, 255);
			} else {
				btn.Ra = (uint8_t) 0;
			}
			break;
		case 5: //Digital => Analog Value + Digital state
			if(hardware.R) {
				btn.Ra = min(((uint8_t) (_RTriggerOffset)) + trigR, 255);
			} else {
				btn.Ra = (uint8_t) 0;
			}
			break;
		default:
			btn.Ra = adc->adc0->analogRead(_pinRa)>>4;
	}

	unsigned int adcCount = 0;
	unsigned int aXSum = 0;
	unsigned int aYSum = 0;
	unsigned int cXSum = 0;
	unsigned int cYSum = 0;

	do{
		adcCount++;
		aXSum += adc->adc0->analogRead(_pinAx);
		aYSum += adc->adc0->analogRead(_pinAy);
		cXSum += adc->adc0->analogRead(_pinCx);
		cYSum += adc->adc0->analogRead(_pinCy);
	}
	while((micros()-_lastMicros) < 1000);


	//Serial.println(adcCount);
	float aStickX = aXSum/(float)adcCount/4096.0*_ADCScale;
	float aStickY = aYSum/(float)adcCount/4096.0*_ADCScale;
	float cStickX = cXSum/(float)adcCount/4096.0*_ADCScale;
	float cStickY = cYSum/(float)adcCount/4096.0*_ADCScale;

	_dT = (micros() - _lastMicros)/1000.0;
	_lastMicros = micros();
	//create the measurement value to be used in the kalman filter
	float xZ;
	float yZ;

	//linearize the analog stick inputs by multiplying by the coefficients found during calibration (3rd order fit)
	xZ = linearize(aStickX,_aFitCoeffsX);
	yZ = linearize(aStickY,_aFitCoeffsY);

	float posCx = linearize(cStickX,_cFitCoeffsX);
	float posCy = linearize(cStickY,_cFitCoeffsY);


	//Run the kalman filter to eliminate snapback
	runKalman(xZ,yZ);
	//Run a simple low-pass filter
	static float _oldPosAx = 0;
	static float _oldPosAy = 0;
	float posAx = _g.xSmoothing*_xPosFilt + (1-_g.xSmoothing)*_oldPosAx;
	float posAy = _g.ySmoothing*_yPosFilt + (1-_g.ySmoothing)*_oldPosAy;
	_oldPosAx = posAx;
	_oldPosAy = posAy;

	//Run a simple low-pass filter on the C-stick
	float oldCX = _cXPos;
	float oldCY = _cYPos;
	_cXPos = posCx;
	_cYPos = posCy;
	float xWeight1 = _g.cXSmoothing;
	float xWeight2 = 1-xWeight1;
	float yWeight1 = _g.cYSmoothing;
	float yWeight2 = 1-yWeight1;

	_cXPos = xWeight1*_cXPos + xWeight2*oldCX;
	_cYPos = yWeight1*_cYPos + yWeight2*oldCY;

	posCx = _cXPos;
	posCy = _cYPos;

	//Run a median filter to reduce noise
#ifdef USEMEDIAN
    runMedian(posAx, _xPosList, _xMedianIndex);
    runMedian(posAy, _yPosList, _yMedianIndex);
#endif

	float remappedAx;
	float remappedAy;
	float remappedCx;
	float remappedCy;
	notchRemap(posAx, posAy, &remappedAx, &remappedAy, _aAffineCoeffs, _aBoundaryAngles,_noOfNotches);
	notchRemap(posCx, posCy, &remappedCx, &remappedCy, _cAffineCoeffs, _cBoundaryAngles,_noOfNotches);

	//Clamp values from -125 to +125
	remappedAx = min(125, max(-125, remappedAx));
	remappedAy = min(125, max(-125, remappedAy));
	remappedCx = min(125, max(-125, remappedCx+_cXOffset));
	remappedCy = min(125, max(-125, remappedCy+_cYOffset));

	float hystVal = 0.3;
	//assign the remapped values to the button struct
	if(readA){
		float diffAx = (remappedAx+127.5)-btn.Ax;
		if( (diffAx > (1.0 + hystVal)) || (diffAx < -hystVal) ){
			btn.Ax = (uint8_t) (remappedAx+127.5);
		}
		float diffAy = (remappedAy+127.5)-btn.Ay;
		if( (diffAy > (1.0 + hystVal)) || (diffAy < -hystVal) ){
			btn.Ay = (uint8_t) (remappedAy+127.5);
		}
	}
	if(readC){
		float diffCx = (remappedCx+127.5)-btn.Cx;
		if( (diffCx > (1.0 + hystVal)) || (diffCx < -hystVal) ){
			btn.Cx = (uint8_t) (remappedCx+127.5);
		}
		float diffCy = (remappedCy+127.5)-btn.Cy;
		if( (diffCy > (1.0 + hystVal)) || (diffCy < -hystVal) ){
			btn.Cy = (uint8_t) (remappedCy+127.5);
		}
	}

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

	if((abs(*xOut)<3) && (abs(*yOut)<3)) {
		*xOut = 0;
		*yOut = 0;
	}
}
/*******************
	setCommResponse
	takes the values that have been put into the button struct and translates them in the serial commands ready
	to be sent to the gamecube/wii
*******************/
void setCommResponse(volatile char response[], Buttons &button){
	for(int i = 0; i < 8; i++){
		//write all of the data in the button struct (taken from the dogebawx project, thanks to GoodDoge)
#ifdef TEENSY3_2
		for(int j = 0; j < 4; j++){
			//this could probably be done better but we need to take 2 bits at a time to put into one serial byte
			//for details on this read here: http://www.qwertymodo.com/hardware-projects/n64/n64-controller
			int these2bits = (button.arr[i]>>(6-j*2)) & 3;
			switch(these2bits){
				case 0:
				response[(i<<2)+j] = 0x08;
				break;
				case 1:
				response[(i<<2)+j] = 0xE8;
				break;
				case 2:
				response[(i<<2)+j] = 0x0F;
				break;
				case 3:
				response[(i<<2)+j] = 0xEF;
				break;
			}
		}
#endif // TEENSY3_2
#ifdef TEENSY4_0
		for(int j = 0; j < 8; j++){
			response[i*8+j] = button.arr[i]>>(7-j) & 1;
		}
#endif // TEENSY4_0
	}
}

#ifdef TEENSY3_2
/*******************
	communicate
	try to communicate with the gamecube/wii
*******************/
void bitCounter(){
	//received a bit of data
	_bitCount ++;
	//digitalWriteFast(12,!(_bitCount%2));
	
	//if this was the first bit of a command we need to set the timer to call communicate() in ~40us when the first 8 bits of the command is received and set the status to reading
	if(_bitCount == 1){
		timer1.trigger((_cmdLengthShort-1)*10);
		_commStatus = _commRead;
	}
}
void communicate(){
	//Serial.println(_commStatus,DEC);
	
	//check to see if we are reading a command
	if(_commStatus == _commRead){
		//wait until we have all 4 serial bytes (8 bits) ready to read
		while(Serial2.available() < (_cmdLengthShort-1)){}
		
		//read the command in
		int cmdByte = 0;
		int cmd = 0;
		bool bitOne = 0;
		bool bitTwo = 0;
		for(int i = 0; i < _cmdLengthShort-1; i++){
			cmd = Serial2.read();
			//the first bit is encoded in the second half of the serial byte, apply a mask to the 6th bit of the serial byte to get it
			bitOne = cmd & 0b00000010;
			//the second bit is encoded in the first half of the serial byte, apply a mask to the 2nd bit of the serial byte to get it
			bitTwo = cmd & 0b01000000;
			//put the two bits into the command byte
			cmdByte = (cmdByte<<1)+bitOne;
			cmdByte = (cmdByte<<1)+bitTwo;

		}
		Serial.print("cmd: ");
		Serial.println(cmdByte,BIN);
		//switch the serial hardware to the faster baud rate to be ready to respond
		setSerialFast();

		switch(cmdByte){
			
		//probe
		case 0x00:
			//set the timer to call communicate() again in ~96 us when the probe response is done being sent
			timer1.trigger(_probeLength*8);
			//write the probe response
			for(int i = 0; i< _probeLength; i++){
				Serial2.write(_probeResponse[i]);
			}
			//write a stop bit
			Serial2.write(0xFF);
			//set the write queue to the sum of the probe command BIT length and the probe response BIT length so we will be about to know when the correct number of bits has been sent
			_writeQueue = _cmdLengthShort*2-1+(_probeLength)*2+1;
			//set the status to writing
			_commStatus = _commWrite;
			Serial.println("probe");
		break;
		
		//origin
		case 0x41:
			//set the timer to call communicate() again in ~320 us when the probe response is done being sent
			timer1.trigger(_originLength*8);
			//write the origin response
			for(int i = 0; i< _originLength; i++){
				Serial2.write(_originResponse[i]);
			}
			//write a stop bit
			Serial2.write(0xFF);
			//set the write queue to the sum of the origin command BIT length and the origin response BIT length so we will be about to know when the correct number of bits has been sent
			_writeQueue = _cmdLengthShort*2-1+(_originLength)*2+1;
			//set the status to writing
			_commStatus = _commWrite;
			Serial.println("origin");
		  break;
			
		//poll
		case 0x40:
			//set the timer to call communicate() again in ~56 us when the poll command is finished being read
			timer1.trigger(56);
			//set the status to receiving the poll command
			_commStatus = _commPoll;
			//create the poll response
			setCommResponse(_commResponse, _btn);
			break;
		default:
		  //got something strange, try waiting for a stop bit to syncronize
			Serial.println("error");
			Serial.println(_bitCount,DEC);

			resetSerial();

			uint8_t thisbyte = 0;
			//wait until we get a stop bit
		  while(thisbyte != 0xFF){
				while(!Serial2.available());
				thisbyte = Serial2.read();
				//Serial.println(thisbyte,BIN);
			}
			//set the status to idle, reset the bit count and write queue so we are ready to receive the next command
			_commStatus = _commIdle;
			_bitCount = 0;
			_writeQueue = 0;
	  }
	}
	else if(_commStatus == _commPoll){
		//we should now be finishing reading the poll command (which is longer than the others)
		while(_bitCount<(_cmdLengthLong*2-1)){} //wait until we've received 25 bits, the length of the poll command
		
		//write the poll response (we do this before setting the timer  here to start responding as soon as possible)
		for(int i = 0; i< _pollLength; i++){
			Serial2.write(_commResponse[i]);
		}
		//write a stop bit
		Serial2.write(0xFF);
		
		//set the timer so communicate() is called in ~135us when the poll response is done being written
		timer1.trigger(135);
		//set the write queue to the sum of the origin command BIT length and the origin response BIT length so we will be about to know when the correct number of bits has been sent
		_writeQueue = _cmdLengthLong*2-1+(_pollLength)*2+1;
		//set the status to writing
		_commStatus = _commWrite;
	}
	else if(_commStatus == _commWrite){
		//wait until we've written all the bits we intend to
 		while(_writeQueue > _bitCount){}
		
		//reset the serial to the slow baudrate
		resetSerial();
		//set the status to idle, reset the bit count and write queue so we are ready to receive the next command
		_bitCount = 0;
		_commStatus = _commIdle;
		_writeQueue = 0;
	}
	else{
		Serial.println("communication status error");
	}
}
void setSerialFast(){
	UART1_BDH = _fastBDH;
	UART1_BDL = _fastBDL;
	UART1_C4 = _fastC4;
	UART1_C2 &= ~UART_C2_RE;
}
void resetSerial(){
	UART1_BDH = _slowBDH;
	UART1_BDL = _slowBDL;
	UART1_C4 = _slowC4;
	UART1_C2 |= UART_C2_RE;
	Serial2.clear();
}
#endif // TEENSY3_2
/*******************
	cleanCalPoints
	take the x and y coordinates and notch angles collected during the calibration procedure,
	and generate the cleaned (non-redundant) x and y stick coordinates and the corresponding x and y notch coordinates
*******************/
void cleanCalPoints(const float calPointsX[], const float calPointsY[], const float notchAngles[], float cleanedPointsX[], float cleanedPointsY[], float notchPointsX[], float notchPointsY[], int notchStatus[]){

	Serial.println("The raw calibration points (x,y) are:");
	for(int i = 0; i< _noOfCalibrationPoints; i++){
		Serial.print(calPointsX[i], 4);
		Serial.print(",");
		Serial.println(calPointsY[i], 4);
	}

	Serial.println("The notch angles are:");
	for(int i = 0; i< _noOfNotches; i++){
		Serial.println(notchAngles[i], 4);
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

		//copy the cal point into the cleaned list
		cleanedPointsX[i+1] = calPointsX[i*2+1];
		cleanedPointsY[i+1] = calPointsY[i*2+1];

		//convert notch angles to x/y coords (weird since the stick moves spherically)
		calcStickValues(notchAngles[i], notchPointsX+i+1, notchPointsY+i+1);
		notchPointsX[i+1] = round(notchPointsX[i+1]);
		notchPointsY[i+1] = round(notchPointsY[i+1]);

		Serial.print(notchPointsX[i+1]);
		Serial.print(",");
		Serial.println(notchPointsY[i+1]);
	}

	//remove the largest and smallest two origin values to remove outliers
	//first, find their indices
	int smallestX = 0;
	int smallX = 0;
	int largeX = 0;
	int largestX = 0;
	int smallestY = 0;
	int smallY = 0;
	int largeY = 0;
	int largestY = 0;
	for (int i = 0; i < _noOfNotches; i++){
		if (calPointsX[i*2] < calPointsX[smallestX]){//if it's the new smallest
			smallX = smallestX;//shuffle the old smallest to small
			smallestX = i*2;//record the new smallest index
		} else if (calPointsX[i*2] < calPointsX[smallX]){//if it's the new second-smallest
			smallX = i*2;//record the new small index
		}
		if (calPointsX[i*2] > calPointsX[largestX]){//if it's the new largest
			largeX = largestX;//shuffle the old largest to large
			largestX = i*2;//record the new largest index
		} else if (calPointsX[i*2] > calPointsX[largeX]){//if it's the new second-largest
			largeX = i*2;//record the new large index
		}
		if (calPointsY[i*2] < calPointsY[smallestY]){
			smallY = smallestY;
			smallestY = i*2;
		} else if (calPointsY[i*2] < calPointsY[smallY]){
			smallY = i*2;
		}
		if (calPointsY[i*2] > calPointsY[largestY]){
			largeY = largestY;
			largestY = i*2;
		} else if (calPointsY[i*2] > calPointsY[largeY]){
			largeY = i*2;
		}
	}
	//subtract the smallest and largest values
	cleanedPointsX[0] -= calPointsX[smallestX];
	cleanedPointsX[0] -= calPointsX[smallX];
	cleanedPointsX[0] -= calPointsX[largeX];
	cleanedPointsX[0] -= calPointsX[largestX];
	cleanedPointsY[0] -= calPointsY[smallestY];
	cleanedPointsY[0] -= calPointsY[smallY];
	cleanedPointsY[0] -= calPointsY[largeY];
	cleanedPointsY[0] -= calPointsY[largestY];

	//divide by the total number of calibration steps/2 to get the average origin value
	//except it's minus 4 steps since we removed outliers
	cleanedPointsX[0] = cleanedPointsX[0]/((float)_noOfNotches-4);
	cleanedPointsY[0] = cleanedPointsY[0]/((float)_noOfNotches-4);

	for(int i = 0; i < _noOfNotches; i++){
		//calculate radius of cleaned point from center
		float deltaX = cleanedPointsX[i+1] - cleanedPointsX[0];
		float deltaY = cleanedPointsY[i+1] - cleanedPointsY[0];
		float mag = sqrt(deltaX*deltaX + deltaY*deltaY);

		if(mag < 0.02){//if the cleaned point was at the center
			//average the previous and next points (cardinal & diagonal) for some sanity
			//note: this will likely bork if this happens to a cardinal or diagonal
			int prevIndex = ((i-1+_noOfNotches) % _noOfNotches) + 1;
			int nextIndex = ((i+1) % _noOfNotches) + 1;

			cleanedPointsX[i+1] = (cleanedPointsX[prevIndex] + cleanedPointsX[nextIndex])/2.0;
			cleanedPointsY[i+1] = (cleanedPointsY[prevIndex] + cleanedPointsY[nextIndex])/2.0;

			notchPointsX[i+1] = (notchPointsX[prevIndex] + notchPointsX[nextIndex])/2.0;
			notchPointsY[i+1] = (notchPointsY[prevIndex] + notchPointsY[nextIndex])/2.0;

			Serial.print("no input was found for notch: ");
			Serial.println(i+1);

			//Mark that notch adjustment should be skipped for this
			notchStatus[i] = _tertiaryNotchInactive;
		}else{
			notchStatus[i] = _notchStatusDefaults[i];
		}
	}

	Serial.println("The cleaned calibration points are:");
	for(int i = 0; i< (_noOfNotches+1); i++){
		Serial.print(cleanedPointsX[i], 4);
		Serial.print(",");
		Serial.println(cleanedPointsY[i], 4);
	}

	Serial.println("The corresponding notch points are:");
	for(int i = 0; i< (_noOfNotches+1); i++){
		Serial.print(notchPointsX[i]);
		Serial.print(",");
		Serial.println(notchPointsY[i]);
	}

	Serial.println("The notch statuses are:");
	for(int i = 0; i< (_noOfNotches); i++){
		Serial.println(notchStatus[i]);
	}
}
//adjustNotch is used to adjust the angles of the notch.
//It is run after calibration points are collected.
//The notch adjustment is limited in order to control
//1. displacement of points (max 12 units out of +/- 100, for now)
//2. stretching of coordinates (max +/- 30%)
void adjustNotch(int currentStepIn, float loopDelta, bool CW, bool CCW, bool reset, bool calibratingAStick, float measuredNotchAngles[], float notchAngles[], int notchStatus[]){
	//This gets run after all the calibration points are collected
	//So we subtract the number of calibration points and switch over to notch adjust order
	const int notchIndex = _notchAdjOrder[currentStepIn-_noOfCalibrationPoints];

	//display the desired value on the other stick
	float x = 0;
	float y = 0;
	calcStickValues(measuredNotchAngles[notchIndex], &x, &y);
	if(calibratingAStick){
		_btn.Cx = (uint8_t) (x + 127.5);
		_btn.Cy = (uint8_t) (y + 127.5);
	}else{
		_btn.Ax = (uint8_t) (x + 127.5);
		_btn.Ay = (uint8_t) (y + 127.5);
	}

	//do nothing if it's not a valid notch to calibrate
	//it'll skip them anyway but just in case
	if(notchStatus[notchIndex] == _tertiaryNotchInactive){
		return;
	}

	//Adjust notch angle according to which button is pressed (do nothing for both buttons)
	if(CW && !CCW){
		notchAngles[notchIndex] += loopDelta*0.000075;
	}else if(CCW && !CW){
		notchAngles[notchIndex] -= loopDelta*0.000075;
	}else if(reset){
		notchAngles[notchIndex] = measuredNotchAngles[notchIndex];
	}else{
		return;
	}

	//Limit the notch adjustment

	//Start out with the limits being 12 units around the circle at the gate
	/*this may be unnecessary in our case, because 12 units is also the 30% stretch limit
	float lowerPosLimit = measuredNotchAngles[notchIndex] - 12/100.f;
	float upperPosLimit = measuredNotchAngles[notchIndex] + 12/100.f;
	if(upperPosLimit < lowerPosLimit){
		upperPosLimit += 2*M_PI;
	}
	*/

	//Now we need to determine the stretch/compression limit
	//Figure out the previous and next notch angles.
	//For most they're the adjacent notches.
	int prevIndex = (notchIndex-1+_noOfNotches) % _noOfNotches;
	int nextIndex = (notchIndex+1) % _noOfNotches;
	//For diagonals, the cardinals are the index points.
	if((notchIndex - 2) % 4 == 0){
		prevIndex = (notchIndex-2+_noOfNotches) % _noOfNotches;
		nextIndex = (notchIndex+2) % _noOfNotches;
	}
	float prevAngle = notchAngles[prevIndex];
	float nextAngle = notchAngles[nextIndex];
	if(nextAngle < prevAngle){
		nextAngle += 2*M_PI;
	}
	float prevMeasAngle = measuredNotchAngles[prevIndex];
	float thisMeasAngle = measuredNotchAngles[notchIndex];
	float nextMeasAngle = measuredNotchAngles[nextIndex];
	if(nextMeasAngle < thisMeasAngle){
		nextMeasAngle += 2*M_PI;
	}
	float lowerStretchLimit = max(prevAngle + 0.7*(thisMeasAngle-prevMeasAngle), nextAngle - 1.3*(nextMeasAngle-thisMeasAngle));
	float upperStretchLimit = min(prevAngle + 1.3*(thisMeasAngle-prevMeasAngle), nextAngle - 0.7*(nextMeasAngle-thisMeasAngle));
	if(upperStretchLimit < lowerStretchLimit){
		upperStretchLimit += 2*M_PI;
	}

	//Combine the limits
	float lowerLimit = lowerStretchLimit;//max(lowerStretchLimit, lowerPosLimit);
	float upperLimit = upperStretchLimit;//min(upperStretchLimit, upperPosLimit);
	if(upperLimit < lowerLimit){
		upperLimit += 2*M_PI;
	}

	//Apply the limits
	notchAngles[notchIndex] = max(notchAngles[notchIndex], lowerLimit);
	notchAngles[notchIndex] = min(notchAngles[notchIndex], upperLimit);
}
//displayNotch is used in lieu of adjustNotch when doing basic calibration
void displayNotch(const int currentStepIn, const bool calibratingAStick, const float notchAngles[]){
	int currentStep = _calOrder[currentStepIn];
	//display the desired value on the other stick
	float x = 0;
	float y = 0;
	if(currentStep%2){
		const int notchIndex = currentStep/2;
		calcStickValues(notchAngles[notchIndex], &x, &y);
	}
	if(calibratingAStick){
		_btn.Cx = (uint8_t) (x + 127.5);
		_btn.Cy = (uint8_t) (y + 127.5);
	}else{
		_btn.Ax = (uint8_t) (x + 127.5);
		_btn.Ay = (uint8_t) (y + 127.5);
	}
}
void collectCalPoints(bool aStick, int currentStepIn, float calPointsX[], float calPointsY[]){
	Serial.print("Collecting cal point for step: ");
	Serial.println(currentStepIn);
    const int currentStep = _calOrder[currentStepIn];

	Serial.print("Cal point number: ");
	Serial.println(currentStep);
	float X;
	float Y;

	for(int j = 0; j < MEDIANLEN; j++){
		X = 0;
		Y = 0;
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
		X = X/128.0;
		Y = Y/128.0;

#ifdef USEMEDIAN
		runMedian(X, _xPosList, _xMedianIndex);
    runMedian(Y, _yPosList, _yMedianIndex);
#endif
	}

	calPointsX[currentStep] = X;
	calPointsY[currentStep] = Y;

	Serial.println("The collected coordinates are: ");
	Serial.println(calPointsX[currentStep],8);
	Serial.println(calPointsY[currentStep],8);
}
/*******************
	linearizeCal
	Generate a fit to linearize the stick response.
	Inputs:
		cleaned points X and Y, (must be 17 points for each of these, the first being the center, the others starting at 3 oclock and going around counterclockwise)
	Outputs:
		linearization fit coefficients for X and Y
*******************/
void linearizeCal(float inX[],float inY[],float outX[], float outY[], float fitCoeffsX[], float fitCoeffsY[]){
	Serial.println("beginning linearization");

	//do the curve fit first
	//generate all the notched/not notched specific cstick values we will need

	double fitPointsX[5];
	double fitPointsY[5];

	fitPointsX[0] = inX[8+1];                   //right
	fitPointsX[1] = (inX[6+1] + inX[10+1])/2.0; //right 45 deg
	fitPointsX[2] = inX[0];                     //center
	fitPointsX[3] = (inX[2+1] + inX[14+1])/2.0; //left 45 deg
	fitPointsX[4] = inX[0+1];                   //left

	fitPointsY[0] = inY[12+1];                  //down
	fitPointsY[1] = (inY[10+1] + inY[14+1])/2.0;//down 45 deg
	fitPointsY[2] = inY[0];                     //center
	fitPointsY[3] = (inY[6+1] + inY[2+1])/2.0;  //up 45 deg
	fitPointsY[4] = inY[4+1];                   //up


	//////determine the coefficients needed to linearize the stick
	//create the expected output, what we want our curve to be fit too
	//this is hard coded because it doesn't depend on the notch adjustments
	//                   -100 -74.246        0     74.246         100, centered around 0-255
    //It's not sin(45 deg) because it's a spherical motion, not planar.
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

	fitCurve(_fitOrder, 5, fitPointsX, x_output, _fitOrder+1, tempCoeffsX);
	fitCurve(_fitOrder, 5, fitPointsY, y_output, _fitOrder+1, tempCoeffsY);

	//write these coefficients to the array that was passed in, this is our first output
	for(int i = 0; i < (_fitOrder+1); i++){
		fitCoeffsX[i] = tempCoeffsX[i];
		fitCoeffsY[i] = tempCoeffsY[i];
	}

	//we will now take out the offset, making the range -100 to 100 instead of 28 to 228
	//calculate the offset
	float xZeroError = linearize((float)fitPointsX[2],fitCoeffsX);
	float yZeroError = linearize((float)fitPointsY[2],fitCoeffsY);

	//Adjust the fit's constant coefficient so that the stick zero position is 0
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
void recomputeGains(){
    //Recompute the intermediate gains used directly by the kalman filter
    //This happens according to the time between loop iterations.
    //Before, this happened every iteration of runKalman, but now
    //the event loop runs at a fixed 1000 Hz
    //Even if it's not *exactly* 1000 Hz, it should be constant enough.
    //Hopefully.
    //So now, this should be called any time _gains gets changed.
    const float timeFactor = 1.0 / 1.2;
    const float timeDivisor = 1.2 / 1.0;
    _g.maxStick      = _gains.maxStick*_gains.maxStick;//we actually use the square
    _g.xVelDecay     = _gains.xVelDecay      * timeFactor;
    _g.yVelDecay     = _gains.yVelDecay      * timeFactor;
    _g.xVelPosFactor = _gains.xVelPosFactor  * timeFactor;
    _g.yVelPosFactor = _gains.yVelPosFactor  * timeFactor;
    _g.xVelDamp      = _gains.xVelDamp       * timeDivisor;
    _g.yVelDamp      = _gains.yVelDamp       * timeDivisor;
    _g.velThresh     = 1/(_gains.velThresh   * timeFactor);//slight optimization by using the inverse
    _g.accelThresh   = 1/(_gains.accelThresh * timeFactor);
    _g.velThresh     = _g.velThresh*_g.velThresh;//square it because it's used squared
    _g.accelThresh   = _g.accelThresh*_g.accelThresh;
    _g.xSmoothing    = pow(1-_gains.xSmoothing, timeDivisor);
    _g.ySmoothing    = pow(1-_gains.ySmoothing, timeDivisor);
    _g.cXSmoothing   = pow(1-_gains.cXSmoothing, timeDivisor);
    _g.cYSmoothing   = pow(1-_gains.cYSmoothing, timeDivisor);
}
void runKalman(const float xZ,const float yZ){
	//Serial.println("Running Kalman");

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
	const float stickDistance2 = min(_g.maxStick, _xPos*_xPos + _yPos*_yPos)/_g.maxStick;//0-1
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

	//But if we _xSnapback or _ySnapback is zero, we skip the calculation
	if(_xSnapback != 0){
		_xVelFilt = velWeight1*_xVel + (1-_g.xVelDecay)*velWeight2*oldXVelFilt + _g.xVelPosFactor*oldXPosDiff;

		const float xPosWeightVelAcc = 1 - min(1, xVelSmooth*xVelSmooth*_g.velThresh + xAccel*xAccel*_g.accelThresh);
		const float xPosWeight1 = max(xPosWeightVelAcc, stickDistance6);
		const float xPosWeight2 = 1-xPosWeight1;

		_xPosFilt = xPosWeight1*_xPos +
		            xPosWeight2*(oldXPosFilt + (1-_g.xVelDamp)*_xVelFilt);
	} else {
		_xPosFilt = _xPos;
	}

	if(_ySnapback != 0){
		_yVelFilt = velWeight1*_yVel + (1-_g.yVelDecay)*velWeight2*oldYVelFilt + _g.yVelPosFactor*oldYPosDiff;

		const float yPosWeightVelAcc = 1 - min(1, yVelSmooth*yVelSmooth*_g.velThresh + yAccel*yAccel*_g.accelThresh);
		const float yPosWeight1 = max(yPosWeightVelAcc, stickDistance6);
		const float yPosWeight2 = 1-yPosWeight1;

		_yPosFilt = yPosWeight1*_yPos +
		            yPosWeight2*(oldYPosFilt + (1-_g.yVelDamp)*_yVelFilt);
	} else {
		_yPosFilt = _yPos;
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

/*
 * calcStickValues computes the stick x/y coordinates from angle.
 * This requires weird trig because the stick moves spherically.
 */
void calcStickValues(float angle, float* x, float* y){
	*x = 100*atan2f((sinf(_maxStickAngle)*cosf(angle)),cosf(_maxStickAngle))/_maxStickAngle;
	*y = 100*atan2f((sinf(_maxStickAngle)*sinf(angle)),cosf(_maxStickAngle))/_maxStickAngle;
}
/*
 * Convert the x/y coordinates (actually angles on a sphere) to an azimuth
 * We first convert to a 3D coordinate and then drop to 2D, then arctan it
 * This does the opposite of calcStickValues, ideally.
 */
void angleOnSphere(const float x, const float y, float& angle){
	float xx = sinf(x*_maxStickAngle/100) * cosf(y*_maxStickAngle/100);
	float yy = cosf(x*_maxStickAngle/100) * sinf(y*_maxStickAngle/100);
	angle = atan2f(yy, xx);//WHY IS THIS BACKWARDS
	if(angle < 0){
		angle += 2*M_PI;
	}
}
/*
 * stripCalPoints removes the notches from un-cleaned cal points
 * this is so we can get the original values of the notches after the affine transform.
 * there need to be _noOfCalibrationPoints values in the inputs and outputs.
 */
void stripCalPoints(float calPointsX[], float calPointsY[], float strippedPointsX[], float strippedPointsY[]){
	for(int i=0; i < _noOfCalibrationPoints; i++){
		//start off by just copying them wholesale
		strippedPointsX[i] = calPointsX[i];
		strippedPointsY[i] = calPointsY[i];
		if((i+1)%4 == 0){//non-cardinal non-diagonal notch (every fourth starting at index 3)
			strippedPointsX[i] = calPointsX[0];//set equal to origin
			strippedPointsY[i] = calPointsY[0];
		}
	}
}
/*
 * transformCalPoints
 * remaps the cleaned calibration points from raw measurements to output coordinates
 * This seems redundant but we're feeding it coordinates without non-diagonal notches
 */
void transformCalPoints(float xInput[], float yInput[], float xOutput[], float yOutput[], float fitCoeffsX[], float fitCoeffsY[], float affineCoeffs[][6], float boundaryAngles[]){
	for(int i=0; i < _noOfNotches+1; i++){
		float xValue = linearize(xInput[i], fitCoeffsX);
		float yValue = linearize(yInput[i], fitCoeffsY);
		float outX;
		float outY;
		notchRemap(xValue, yValue, &outX, &outY, affineCoeffs, boundaryAngles, _noOfNotches);
		xOutput[i] = outX;
		yOutput[i] = outY;
	}
}
/*
 * computeStickAngles
 * write all the stick angles into the notch angles array array
 * inputs need to be length _noOfNotches+1
 * outputs need to be length _noOfNotches
 */
void computeStickAngles(float xInput[], float yInput[], float stickAngles[]){
	Serial.println("Computed stick angles:");
	for(int i=0; i < _noOfNotches; i++){
		if(i%2 == 0){//cardinal or diagonal
			stickAngles[i] = _notchAngleDefaults[i];
		} else {
			angleOnSphere(xInput[i+1], yInput[i+1], stickAngles[i]);
		}
	}
}
//sets notches to measured values if absent
void cleanNotches(float notchAngles[], float measuredNotchAngles[], int notchStatus[]){
	for(int i=0; i < _noOfNotches; i++){
		if(notchStatus[i] == _tertiaryNotchInactive){
			notchAngles[i] = measuredNotchAngles[i];
		}
	}
}
