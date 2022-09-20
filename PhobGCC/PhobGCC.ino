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
	DEFAULTJUMP,
	SWAP_XZ,
	SWAP_YZ
};

enum WhichTrigger {
	LTRIGGER,
	RTRIGGER
};

enum Increase {
	INCREASE,
	DECREASE
};

enum WhichAxis {
	XAXIS,
	YAXIS
};

enum HardReset {
	HARD,
	SOFT
};

//defining control configuration
struct ControlConfig{
	int pinXSwappable;
	int pinYSwappable;
	int pinZSwappable;
	JumpConfig jumpConfig;
	const int jumpConfigMin;
	const int jumpConfigMax;
	int lConfig;
	int rConfig;
	const int triggerConfigMin;
	const int triggerConfigMax;
	const int triggerDefault;
	int lTriggerOffset;
	int rTriggerOffset;
	const int triggerMin;
	const int triggerMax;
	int cXOffset;
	int cYOffset;
	const int cMax;
	const int cMin;
	int rumble;//0 is off, nonzero is on, higher is stronger
	const int rumbleMin;
	const int rumbleMax;
	const int rumbleDefault;
	bool safeMode;
	bool autoInit;
	int lTrigInitial;
	int rTrigInitial;
	int xSnapback;//0 disables the filter entirely, 4 is default
	int ySnapback;
	const int snapbackMin;
	const int snapbackMax;
	const int snapbackDefault;
	const float smoothingMin;
	const float smoothingMax;
};
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

//////values used for calibration
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
//Edit MEDIANARRAY to be MEDIANLEN long
#define MEDIANARRAY {0,0,0,0,0}
//Comment this define to disable it entirely.
//#define USEMEDIAN


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

	_lastMicros = micros();

    setPinModes();

    ADCSetup(adc, _ADCScale, _ADCScaleFactor);

	//measure the trigger values
	initializeButtons(_btn,_controls.lTrigInitial,_controls.rTrigInitial);
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
			readSticks(true,false, _btn, _hardware, _controls, _normGains);
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
			readSticks(false,true, _btn, _hardware, _controls, _normGains);
		}
	}
	else if(_running){
		//if not calibrating read the sticks normally
		readSticks(true,true, _btn, _hardware, _controls, _normGains);
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
			if(_cmdByte & 0b00000001){
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
int readEEPROM(ControlConfig &controls, FilterGains &gains, FilterGains &normGains){
	int numberOfNaN = 0;

	//get the jump setting
	EEPROM.get(_eepromJump, controls.jumpConfig);
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
	EEPROM.get(_eepromLToggle, controls.lConfig);
	if(controls.lConfig < controls.triggerConfigMin) {
		controls.lConfig = controls.triggerDefault;
		numberOfNaN++;
	}
	if(controls.lConfig > controls.triggerConfigMax) {
		controls.lConfig = controls.triggerDefault;
		numberOfNaN++;
	}

	//get the R setting
	EEPROM.get(_eepromRToggle, controls.rConfig);
	if(controls.rConfig < controls.triggerConfigMin) {
		controls.rConfig = controls.triggerDefault;
		numberOfNaN++;
	}
	if(controls.rConfig > controls.triggerConfigMax) {
		controls.rConfig = controls.triggerDefault;
		numberOfNaN++;
	}

	//get the C-stick X offset
	EEPROM.get(_eepromcXOffset, controls.cXOffset);
	if(controls.cXOffset > controls.cMax) {
		controls.cXOffset = 0;
		numberOfNaN++;
	} else if(controls.cXOffset < controls.cMin) {
		controls.cXOffset = 0;
		numberOfNaN++;
	}

	//get the C-stick Y offset
	EEPROM.get(_eepromcYOffset, controls.cYOffset);
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

	//get the L-trigger Offset value
	EEPROM.get(_eepromLOffset, controls.lTriggerOffset);
	if(controls.lTriggerOffset > controls.triggerMax) {
		controls.lTriggerOffset = controls.triggerMax;
		numberOfNaN++;
	} else if(controls.lTriggerOffset < controls.triggerMin) {
		controls.lTriggerOffset = controls.triggerMin;
		numberOfNaN++;
	}

	//get the R-trigger Offset value
	EEPROM.get(_eepromROffset, controls.rTriggerOffset);
	if(controls.rTriggerOffset > controls.triggerMax) {
		controls.rTriggerOffset = controls.triggerMax;
		numberOfNaN++;
	} else if(controls.rTriggerOffset < controls.triggerMin) {
		controls.rTriggerOffset = controls.triggerMin;
		numberOfNaN++;
	}

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
	EEPROM.put(_eepromJump,controls.jumpConfig);

	controls.lConfig = controls.triggerDefault;
	controls.rConfig = controls.triggerDefault;
	EEPROM.put(_eepromLToggle, controls.lConfig);
	EEPROM.put(_eepromRToggle, controls.rConfig);

	controls.cXOffset = 0;
	controls.cYOffset = 0;
	EEPROM.put(_eepromcXOffset, controls.cXOffset);
	EEPROM.put(_eepromcYOffset, controls.cYOffset);

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
	EEPROM.put(_eepromLOffset, controls.lTriggerOffset);
	EEPROM.put(_eepromROffset, controls.rTriggerOffset);

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

	bounceDr.attach(_pinDr);
	bounceDr.interval(1000);
	bounceDu.attach(_pinDu);
	bounceDu.interval(1000);
	bounceDl.attach(_pinDl);
	bounceDl.interval(1000);
	bounceDd.attach(_pinDd);
	bounceDd.interval(1000);
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
		EEPROM.put(_eepromcXOffset, controls.cXOffset);
		Serial.print("X offset increased to:");
		Serial.println(controls.cXOffset);
	} else if(axis == XAXIS && increase == DECREASE) {
		controls.cXOffset--;
		if(controls.cXOffset < controls.cMin) {
			controls.cXOffset = controls.cMin;
		}
		EEPROM.put(_eepromcXOffset, controls.cXOffset);
		Serial.print("X offset decreased to:");
		Serial.println(controls.cXOffset);
	} else if(axis == YAXIS && increase == INCREASE) {
		controls.cYOffset++;
		if(controls.cYOffset > controls.cMax) {
			controls.cYOffset = controls.cMax;
		}
		EEPROM.put(_eepromcYOffset, controls.cYOffset);
		Serial.print("Y offset increased to:");
		Serial.println(controls.cYOffset);
	} else if(axis == YAXIS && increase == DECREASE) {
		controls.cYOffset--;
		if(controls.cYOffset < controls.cMin) {
			controls.cYOffset = controls.cMin;
		}
		EEPROM.put(_eepromcYOffset, controls.cYOffset);
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

	EEPROM.put(_eepromLOffset, controls.lTriggerOffset);
	EEPROM.put(_eepromROffset, controls.rTriggerOffset);

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
	EEPROM.put(_eepromJump,controls.jumpConfig);
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
	EEPROM.put(_eepromLToggle, controls.lConfig);
	EEPROM.put(_eepromRToggle, controls.rConfig);

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
void readSticks(int readA, int readC, Buttons &btn, const HardwareButtons &hardware, const ControlConfig &controls, const FilterGains &normGains){
#ifdef USEADCSCALE
	_ADCScale = _ADCScale*0.999 + _ADCScaleFactor/adc->adc1->analogRead(ADC_INTERNAL_SOURCE::VREF_OUT);
#endif
	// otherwise _ADCScale is 1

	//read the L and R sliders

	//set up lockout for mode 5; it's not permissible to have analog trigger
	// inputs available while mode 5 is active
	//when a trigger is in lockout due to the other being mode 5,
	// modes 1, 3, and 4 will have no output on that trigger to warn the user.
	//(the above modes are 1-indexed, user-facing values)
	const bool lockoutL = controls.rConfig == 4;
	const bool lockoutR = controls.lConfig == 4;

	if(hardware.L && hardware.R && btn.A && btn.S) {
		btn.L = (uint8_t) (1);
		btn.R = (uint8_t) (1);
		btn.A = (uint8_t) (1);
		btn.S = (uint8_t) (1);
	} else {
		switch(controls.lConfig) {
			case 0: //Default Trigger state
				if(lockoutL){
					btn.L  = (uint8_t) 0;
					btn.La = (uint8_t) 0;
				} else {
					btn.L  = hardware.L;
					btn.La = adc->adc0->analogRead(_pinLa)>>4;
				}
				break;
			case 1: //Digital Only Trigger state
				btn.L  = hardware.L;
				btn.La = (uint8_t) 0;
				break;
			case 2: //Analog Only Trigger state
				if(lockoutL){
					btn.L  = (uint8_t) 0;
					btn.La = (uint8_t) 0;
				} else {
					btn.L  = (uint8_t) 0;
					btn.La = adc->adc0->analogRead(_pinLa)>>4;
				}
				break;
			case 3: //Trigger Plug Emulation state
				if(lockoutL){
					btn.L  = (uint8_t) 0;
					btn.La = (uint8_t) 0;
				} else {
					btn.L  = hardware.L;
					btn.La = adc->adc0->analogRead(_pinLa)>>4;
					if (btn.La > (((uint8_t) (controls.lTriggerOffset)) + controls.lTrigInitial)) {
						btn.La = (((uint8_t) (controls.lTriggerOffset)) + controls.lTrigInitial);
					}
				}
				break;
			case 4: //Digital => Analog Value state
				btn.L = (uint8_t) 0;
				if(hardware.L) {
					btn.La = min(((uint8_t) (controls.lTriggerOffset)) + controls.lTrigInitial, 255);
				} else {
					btn.La = (uint8_t) 0;
				}
				break;
			case 5: //Digital => Analog Value + Digital state
				btn.L = hardware.L;
				if(hardware.L) {
					btn.La = min(((uint8_t) (controls.lTriggerOffset)) + controls.lTrigInitial, 255);
				} else {
					btn.La = (uint8_t) 0;
				}
				break;
			default:
				if(lockoutL){
					btn.L  = (uint8_t) 0;
					btn.La = (uint8_t) 0;
				} else {
					btn.L  = hardware.L;
					btn.La = adc->adc0->analogRead(_pinLa)>>4;
				}
		}

		switch(controls.rConfig) {
			case 0: //Default Trigger state
				if(lockoutR){
					btn.R  = (uint8_t) 0;
					btn.Ra = (uint8_t) 0;
				} else {
					btn.R  = hardware.R;
					btn.Ra = adc->adc0->analogRead(_pinRa)>>4;
				}
				break;
			case 1: //Digital Only Trigger state
				btn.R  = hardware.R;
				btn.Ra = (uint8_t) 0;
				break;
			case 2: //Analog Only Trigger state
				if(lockoutR){
					btn.R  = (uint8_t) 0;
					btn.Ra = (uint8_t) 0;
				} else {
					btn.R  = (uint8_t) 0;
					btn.Ra = adc->adc0->analogRead(_pinRa)>>4;
				}
				break;
			case 3: //Trigger Plug Emulation state
				if(lockoutR){
					btn.R  = (uint8_t) 0;
					btn.Ra = (uint8_t) 0;
				} else {
					btn.R  = hardware.R;
					btn.Ra = adc->adc0->analogRead(_pinRa)>>4;
					if (btn.Ra > (((uint8_t) (controls.rTriggerOffset)) + controls.rTrigInitial)) {
						btn.Ra = (((uint8_t) (controls.rTriggerOffset)) + controls.rTrigInitial);
					}
				}
				break;
			case 4: //Digital => Analog Value state
				btn.R = (uint8_t) 0;
				if(hardware.R) {
					btn.Ra = min(((uint8_t) (controls.rTriggerOffset)) + controls.rTrigInitial, 255);
				} else {
					btn.Ra = (uint8_t) 0;
				}
				break;
			case 5: //Digital => Analog Value + Digital state
				btn.R = hardware.R;
				if(hardware.R) {
					btn.Ra = min(((uint8_t) (controls.rTriggerOffset)) + controls.rTrigInitial, 255);
				} else {
					btn.Ra = (uint8_t) 0;
				}
				break;
			default:
				if(lockoutR){
					btn.R  = (uint8_t) 0;
					btn.Ra = (uint8_t) 0;
				} else {
					btn.R  = hardware.R;
					btn.Ra = adc->adc0->analogRead(_pinRa)>>4;
				}
		}
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
	static float xPosFilt = 0;//output of kalman filter
	static float yPosFilt = 0;//output of kalman filter
	runKalman(xPosFilt, yPosFilt, xZ, yZ, controls, normGains);
	//Run a simple low-pass filter
	static float oldPosAx = 0;
	static float oldPosAy = 0;
	float posAx = normGains.xSmoothing*xPosFilt + (1-normGains.xSmoothing)*oldPosAx;
	float posAy = normGains.ySmoothing*yPosFilt + (1-normGains.ySmoothing)*oldPosAy;
	oldPosAx = posAx;
	oldPosAy = posAy;

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
	notchRemap(posAx, posAy, &remappedAx, &remappedAy, _aAffineCoeffs, _aBoundaryAngles,_noOfNotches);
	notchRemap(posCx, posCy, &remappedCx, &remappedCy, _cAffineCoeffs, _cBoundaryAngles,_noOfNotches);

	//Clamp values from -125 to +125
	remappedAx = min(125, max(-125, remappedAx));
	remappedAy = min(125, max(-125, remappedAy));
	remappedCx = min(125, max(-125, remappedCx+controls.cXOffset));
	remappedCy = min(125, max(-125, remappedCy+controls.cYOffset));

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
void adjustNotch(int currentStepIn, float loopDelta, bool calibratingAStick, float measuredNotchAngles[], float notchAngles[], int notchStatus[], Buttons &btn, HardwareButtons &hardware){
	//set up variables based on current button state
	bool CW = hardware.X;
	bool CCW = hardware.Y;
	bool reset = btn.B;

	//This gets run after all the calibration points are collected
	//So we subtract the number of calibration points and switch over to notch adjust order
	const int notchIndex = _notchAdjOrder[currentStepIn-_noOfCalibrationPoints];

	//display the desired value on the other stick
	float x = 0;
	float y = 0;
	calcStickValues(measuredNotchAngles[notchIndex], &x, &y);
	if(calibratingAStick){
		btn.Cx = (uint8_t) (x + 127.5);
		btn.Cy = (uint8_t) (y + 127.5);
	}else{
		btn.Ax = (uint8_t) (x + 127.5);
		btn.Ay = (uint8_t) (y + 127.5);
	}

	//do nothing if it's not a valid notch to calibrate
	//it'll skip them anyway but just in case
	if(notchStatus[notchIndex] == _tertiaryNotchInactive){
		return;
	}

	//Adjust notch angle according to which button is pressed (do nothing for both buttons)
	if(CW && !CCW){
		notchAngles[notchIndex] -= loopDelta*0.000075;
	}else if(CCW && !CW){
		notchAngles[notchIndex] += loopDelta*0.000075;
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

	bool isDiagonal = false;

	//Now we need to determine the stretch/compression limit
	//Figure out the previous and next notch angles.
	//For most they're the adjacent notches.
	int prevIndex = (notchIndex-1+_noOfNotches) % _noOfNotches;
	int nextIndex = (notchIndex+1) % _noOfNotches;
	//For diagonals, the cardinals are the index points.
	if((notchIndex - 2) % 4 == 0){
		prevIndex = (notchIndex-2+_noOfNotches) % _noOfNotches;
		nextIndex = (notchIndex+2) % _noOfNotches;
		isDiagonal = true;
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

	float lowerCompressLimit = prevAngle + 0.7*(thisMeasAngle-prevMeasAngle);//how far we can squish when reducing the angle
	float lowerStretchLimit  = nextAngle - 1.3*(nextMeasAngle-thisMeasAngle);//how far we can stretch when reducing the angle
	float upperCompressLimit = nextAngle - 0.7*(nextMeasAngle-thisMeasAngle);//how far we can squish when increasing the angle
	float upperStretchLimit  = prevAngle + 1.3*(thisMeasAngle-prevMeasAngle);//how far we can stretch when increasing the angle

	//Now, in order to apply stretch leniency to angles within the deadzone,
	// we need to figure out whether the previous angle or next angle was a cardinal.
	//If the previous one is a cardinal AND the angle is in the deadzone, we make the upperstretchlimit bigger, only if it can't reach 0.3000.
	const float minThreshold  = 0.1500/0.9750;//radians; we don't want to fix things smaller than this
	const float deadzoneLimit = 0.2875/0.9500;//radians; or things larger than this
	const float deadzonePlus  = 0.3000/0.9500;//radians; we want to make sure the adjustment can make it here
	if(prevIndex % 4 == 0 && !isDiagonal && (thisMeasAngle-prevMeasAngle) > minThreshold && (thisMeasAngle-prevMeasAngle) < deadzoneLimit){
		upperStretchLimit = prevAngle + max(1.3*(thisMeasAngle-prevMeasAngle), deadzonePlus);
	}
	//If the next one is a cardinal AND the angle is in the deadzone, we make the lowerstretchlimit smaller.
	if(nextIndex % 4 == 0 && !isDiagonal && (nextMeasAngle-thisMeasAngle) > minThreshold && (nextMeasAngle-thisMeasAngle) < deadzoneLimit){
		lowerStretchLimit = nextAngle - max(1.3*(nextMeasAngle-thisMeasAngle), deadzonePlus);
	}

	float lowerDistortLimit  = max(lowerCompressLimit, lowerStretchLimit);
	float upperDistortLimit  = min(upperCompressLimit, upperStretchLimit);
	if(upperDistortLimit < lowerDistortLimit){
		upperDistortLimit += 2*M_PI;
	}

	//Combine the limits
	float lowerLimit = lowerDistortLimit;//max(lowerStretchLimit, lowerPosLimit);
	float upperLimit = upperDistortLimit;//min(upperStretchLimit, upperPosLimit);
	if(upperLimit < lowerLimit){
		upperLimit += 2*M_PI;
	}

	//Apply the limits
	notchAngles[notchIndex] = max(notchAngles[notchIndex], lowerLimit);
	notchAngles[notchIndex] = min(notchAngles[notchIndex], upperLimit);
}
//displayNotch is used in lieu of adjustNotch when doing basic calibration
void displayNotch(const int currentStepIn, const bool calibratingAStick, const float notchAngles[], Buttons &btn){
	int currentStep = _calOrder[currentStepIn];
	//display the desired value on the other stick
	float x = 0;
	float y = 0;
	if(currentStep%2){
		const int notchIndex = currentStep/2;
		calcStickValues(notchAngles[notchIndex], &x, &y);
	}
	if(calibratingAStick){
		btn.Cx = (uint8_t) (x + 127.5);
		btn.Cy = (uint8_t) (y + 127.5);
	}else{
		btn.Ax = (uint8_t) (x + 127.5);
		btn.Ay = (uint8_t) (y + 127.5);
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
		static float xPosList[MEDIANLEN] = MEDIANARRAY;//for median filtering;
		static float yPosList[MEDIANLEN] = MEDIANARRAY;//for median filtering
		static unsigned int xMedianIndex = 0;
		static unsigned int yMedianIndex = 0;
		runMedian(X, xPosList, xMedianIndex);
		runMedian(Y, yPosList, yMedianIndex);
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
}
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
	const float stickDistance2 = min(normGains.maxStick, xPos*xPos + yPos*yPos)/normGains.maxStick;//0-1
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

		const float xPosWeightVelAcc = 1 - min(1, xVelSmooth*xVelSmooth*normGains.velThresh + xAccel*xAccel*normGains.accelThresh);
		const float xPosWeight1 = max(xPosWeightVelAcc, stickDistance6);
		const float xPosWeight2 = 1-xPosWeight1;

		xPosFilt = xPosWeight1*xPos +
		            xPosWeight2*(oldXPosFilt + (1-normGains.xVelDamp)*xVelFilt);
	} else {
		xPosFilt = xPos;
	}

	if(controls.ySnapback != 0){
		yVelFilt = velWeight1*yVel + (1-normGains.yVelDecay)*velWeight2*oldYVelFilt + normGains.yVelPosFactor*oldYPosDiff;

		const float yPosWeightVelAcc = 1 - min(1, yVelSmooth*yVelSmooth*normGains.velThresh + yAccel*yAccel*normGains.accelThresh);
		const float yPosWeight1 = max(yPosWeightVelAcc, stickDistance6);
		const float yPosWeight2 = 1-yPosWeight1;

		yPosFilt = yPosWeight1*yPos +
		            yPosWeight2*(oldYPosFilt + (1-normGains.yVelDamp)*yVelFilt);
	} else {
		yPosFilt = yPos;
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
