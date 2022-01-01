//uses a teensy 3.2, hardware serial pins 0 (RX) and 1(TX), diode from 0 to 1 to make the output half duplex, data line connected to pin 0(RX)
//read this for a description of how this works:

#include <math.h>
#include <curveFitting.h>
#include <EEPROM.h>
#include <eigen.h>
#include <Eigen/LU>
#include <Eigen/Dense>

using namespace Eigen;
#define X_ADC_VAR_PEAK 50
#define X_ADC_VAR_SPREAD 1000
#define X_DAMPING 0.0005
#define X_ACCEL_VAR 0.000000000001
#define X_VEL_THRESH 

//#define Y_ADC_VAR_PEAK 50
//#define Y_ADC_VAR_SPREAD 1000
#define Y_DAMPING 0.0005
#define Y_ACCEL_VAR 0.000000000001
#define Y_VEL_THRESH 


#define CMD_LENGTH_SHORT 5
#define CMD_LENGTH_LONG 13
#define ORIGIN_LENGTH 41
#define POLL_LENGTH 33
#define CALIBRATION_POINTS 17

#define HALX 21
#define HALY 20

#define FIT_ORDER 3


#define GC_FREQUENCY 1250000
#define PC_FREQUENCY 1000000
#define PULSE_FREQ_CUTOFF 291666

#define GATE_REGIONS 8

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

float AStickHX;
float AStickHY;
int writeQueue;
//float xCoeffs[FIT_ORDER+1] = {0,0,1,0};
//float yCoeffs[FIT_ORDER+1] = {0,0,1,0};
//float xCoeffs[FIT_ORDER+1] = {-0.01,-1.41,236};
//float yCoeffs[FIT_ORDER+1] = {-0.01,3.11,-68.38};
//float fitCoeffs[(FIT_ORDER+1)*2] = {0,0,0,0.03125,0,  0,0,0,0.03125,0};
float fitCoeffs[(FIT_ORDER+1)*2] = {0,0,0.03125,0,  0,0,0.03125,0};
int calStep;
unsigned int startBtnSince;
int lastStartBtn;
int watchingStart;
float calPointsX[CALIBRATION_POINTS];
float calPointsY[CALIBRATION_POINTS];
unsigned int lastMicros;
bool running;

VectorXf xState(2);
VectorXf yState(2);
MatrixXf xP(2,2);
MatrixXf yP(2,2);
MatrixXf Fmat(2,2);
MatrixXf xQ(2,2);
MatrixXf yQ(2,2);
float xAccelVar;
float yAccelVar;
float damping;
float storedAffineCoeffs[GATE_REGIONS][6] = {1,0,0,0,1,0,
																			 1,0,0,0,1,0,
																			 1,0,0,0,1,0,
																			 1,0,0,0,1,0,
																			 1,0,0,0,1,0,
																			 1,0,0,0,1,0,
																			 1,0,0,0,1,0,
																			 1,0,0,0,1,0,};
float angles[GATE_REGIONS] = {0,M_PI/4,M_PI/2,3*M_PI/4,M_PI,5*M_PI/4,3*M_PI/2,7*M_PI/4};


static uint8_t probeResponse[CMD_LENGTH_LONG] = {
	0x08,0x08,0x0F,0xE8,
	0x08,0x08,0x08,0x08,
	0x08,0x08,0x08,0xEF,
	0xFF};
volatile uint8_t pollResponse[POLL_LENGTH] = {
0x08,0x08,0x08,0x08,
0x0F,0x08,0x08,0x08,
0x08,0x08,0xEF,0xEF,
0x08,0x08,0xEF,0xEF,
0x08,0x08,0xEF,0xEF,
0x08,0x08,0xEF,0xEF,
0x08,0x08,0xEF,0xEF,
0x08,0x08,0xEF,0xEF,
0xFF};
static uint8_t originResponse[ORIGIN_LENGTH] = {
	0x08,0x08,0x08,0x08,
	0x0F,0x08,0x08,0x08,
	0x08,0x08,0xEF,0xEF,
	0x08,0x08,0xEF,0xEF,
	0x08,0x08,0xEF,0xEF,
	0x08,0x08,0xEF,0xEF,
	0x08,0x08,0xEF,0xEF,
	0x08,0x08,0xEF,0xEF,
	0x08,0x08,0x08,0x08,
	0x08,0x08,0x08,0x08,
	0xFF};

int cmd[CMD_LENGTH_LONG];
uint8_t cmdByte;

void setup() {
	//try to determine the speed the hardware serial needs to run at by counting the probe command pulse widths
	noInterrupts();
	unsigned int counter = 0;
	unsigned int start = 0;
	unsigned int duration = 0;
	ARM_DEMCR |= ARM_DEMCR_TRCENA;
	ARM_DWT_CTRL |= ARM_DWT_CTRL_CYCCNTENA;
	//attempt to count a bunch of 0 bits (will also catch some stop bits which are short, will skew the results)
	for(int i = 0; i<64;i++){
		//wait for the pause between pulses
		start = ARM_DWT_CYCCNT;
		duration = 0;
		while(duration<960){
			if(digitalReadFast(0)){
				duration = ARM_DWT_CYCCNT-start;
			}
			else{
				start = ARM_DWT_CYCCNT;
				duration = 0;
			}
		}
		
		//measure the clock cycles of the first 0 bit
		while(digitalReadFast(0)){}
		start = ARM_DWT_CYCCNT;
		while(!digitalReadFast(0)){}
		counter += ARM_DWT_CYCCNT-start;
	}
	counter = counter>>6;
	pulseWidthFreq = F_CPU/counter;
	int serialFreq;
	//pulse widths on my usb adapter are ~4us, gamecube/wii's are supposed to be 3us, this should check directly between them
	if(pulseWidthFreq < PULSE_FREQ_CUTOFF){
		serialFreq = PC_FREQUENCY;
	}
	else{
		serialFreq = GC_FREQUENCY;
	}
	interrupts();
	
	
	running = false;
	btn.errS = 0;
	btn.errL = 0;
	btn.orig = 0;
	btn.high = 1;
	btn.Ax = 128;
	btn.Ay = 128;
	startBtnSince = millis();
	lastStartBtn = 0;
	watchingStart = 0;
	calStep = -1;
	//EEPROM.put( FIT_ADDRESS, fitCoeffs );
	//float eepromTemp[(FIT_ORDER+1)*2 + GATE_REGIONS*7];
	
	//first (FIT_ORDER+1)*2*4 bytes are the fit coefficients
	EEPROM.get( 0, fitCoeffs );
	//next GATE_REGIONS*6*4 bytes are the affine transformation coefficients
	EEPROM.get( (FIT_ORDER+1)*2*4, storedAffineCoeffs );
	//and last is the angles for the different gate regions
	EEPROM.get( (FIT_ORDER+1)*2*4+GATE_REGIONS*6*4, angles );
	
	lastMicros = micros();
	xAccelVar = X_ACCEL_VAR;
	yAccelVar = Y_ACCEL_VAR;
	xDamping = X_DAMPING;
	yDamping = X_DAMPING;
	
	writeQueue = 0;
	
	xState << 0,0;
	yState << 0,0;
	xP << 1000,0,0,1000;
	yP << 1000,0,0,1000;
	
	adc->adc0->setAveraging(8); // set number of averages
  adc->adc0->setResolution(13); // set bits of resolution
  adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::HIGH_SPEED ); // change the conversion speed
  adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_LOW_SPEED ); // change the sampling speed
	
	setPinModes();
	
	//start USB serial
	Serial.begin(57600);
	//start hardware serail
	Serial1.begin(serialFreq);
	attachInterrupt(0, communicate, FALLING);
}

void loop() {
	readButtons();
	if(btn.A){
		running=true;
	}
	readSticks();
	if(calStep >=0){
		calibrate();
	}
	if(running){
		setPole();
	}
}

//void serialEvent1() {
//	communicate();
//}
void setPinModes(){
	pinMode(9,INPUT_PULLUP);
	pinMode(7,INPUT_PULLUP);
	pinMode(10,INPUT_PULLUP);
	pinMode(12,INPUT_PULLUP);
	pinMode(11,INPUT_PULLUP);
	pinMode(6,INPUT_PULLUP);
	pinMode(13,INPUT_PULLUP);
	pinMode(8,INPUT_PULLUP);
	pinMode(3,INPUT_PULLUP);
	pinMode(4,INPUT_PULLUP);
	pinMode(2,INPUT_PULLUP);
	pinMode(5,INPUT_PULLUP);

	pinMode(18,INPUT);
	pinMode(19,INPUT);
	pinMode(21,INPUT);
	pinMode(20,INPUT);
	pinMode(17,INPUT);
	pinMode(16,INPUT);
}
void readButtons(){
	btn.A = !digitalRead(9);
	btn.B = !digitalRead(7);
	btn.X = !digitalRead(10);
	btn.Y = !digitalRead(12);
	btn.Z = !digitalRead(11);
	btn.S = !digitalRead(6);
	btn.L = !digitalRead(13); //note, need to cut and reroute this trace, was to pin 1
	btn.R = !digitalRead(8);
	btn.Du = !digitalRead(3);
	btn.Dd = !digitalRead(4);
	btn.Dl = !digitalRead(2);
	btn.Dr = !digitalRead(5);
	
	if(btn.Dl && !lastStartBtn){
		startBtnSince = millis();
		watchingStart = true;
	}
	else if(btn.Dl && watchingStart){
		int startTimer = millis()- startBtnSince;
		//Serial.println(startTimer);
		if(startTimer > 1000){
			calStep ++;
			watchingStart = false;	
			Serial.println("calibrating");
			Serial.println(calStep);
		}
	}
	lastStartBtn = btn.Dl;
}
void readSticks(){
	//btn.Ax = analogRead(18);
	//btn.Ay = analogRead(19);

	btn.La = analogRead(22)>>5;
	btn.Ra = analogRead(15)>>5;
	
	//Serial.print("L Trigger");
	//Serial.println(btn.La,DEC);
	//Serial.print("R Trigger");
	//Serial.println(btn.Ra,DEC);
	
	AStickHX = analogRead(HALX)/8192.0;
	AStickHY = analogRead(HALY)/8192.0;
	
	
	//btn.Ax = (uint8_t) (xCoeffs[0]*pow(,3) + xCoeffs[1]*pow(AStickHX,2) + xCoeffs[2]*pow(AStickHX,1) + xCoeffs[2]*AStickHX) + 128;
	//btn.Ay = (uint8_t) (yCoeffs[0]*pow(AStickHY,3) + yCoeffs[1]*pow(AStickHY,2) + yCoeffs[2]*pow(AStickHY,1) + yCoeffs[2]*AStickHY) + 128;
	
	//btn.Ax = (uint8_t) (fitCoeffs[0]*(AStickHX*AStickHX*AStickHX*AStickHX) + fitCoeffs[1]*(AStickHX*AStickHX*AStickHX) + fitCoeffs[2]*(AStickHX*AStickHX) + fitCoeffs[3]*AStickHX + fitCoeffs[4]);
	//btn.Ay = (uint8_t) (fitCoeffs[5]*(AStickHY*AStickHY*AStickHY*AStickHY) + fitCoeffs[6]*(AStickHY*AStickHY*AStickHY) + fitCoeffs[7]*(AStickHY*AStickHY) + fitCoeffs[8]*AStickHY + fitCoeffs[9]);

	//btn.Ax = (uint8_t) (fitCoeffs[0]*(AStickHX*AStickHX) + fitCoeffs[1]*AStickHX + fitCoeffs[2]); //+ 128;
	//btn.Ay = (uint8_t) (fitCoeffs[3]*(AStickHY*AStickHY) + fitCoeffs[4]*AStickHY + fitCoeffs[5]); //+ 128;
	
	VectorXf xZ(1);
	VectorXf yZ(1);
	//xZ << (fitCoeffs[0]*(AStickHX*AStickHX*AStickHX*AStickHX) + fitCoeffs[1]*(AStickHX*AStickHX*AStickHX) + fitCoeffs[2]*(AStickHX*AStickHX) + fitCoeffs[3]*AStickHX + fitCoeffs[4]);
	//yZ << (fitCoeffs[5]*(AStickHY*AStickHY*AStickHY*AStickHY) + fitCoeffs[6]*(AStickHY*AStickHY*AStickHY) + fitCoeffs[7]*(AStickHY*AStickHY) + fitCoeffs[8]*AStickHY + fitCoeffs[9]);
	
	xZ << (fitCoeffs[0]*(AStickHX*AStickHX*AStickHX) + fitCoeffs[1]*(AStickHX*AStickHX) + fitCoeffs[2]*AStickHX + fitCoeffs[3]);
	yZ << (fitCoeffs[4]*(AStickHY*AStickHY*AStickHY) + fitCoeffs[5]*(AStickHY*AStickHY) + fitCoeffs[6]*AStickHY + fitCoeffs[7]);

	//Serial.print(xZ[0],10);
	//Serial.print(',');
	//Serial.println(xZ[1],10);
	//Serial.println(yZ[0]);
	
	runKalman(xZ,yZ);
	
	//Serial.print(xState[0],10);
	//Serial.print(',');
	//Serial.println(xState[1],10);
	
	float angle = atan2f((yZ[0]-128),(xZ[0]-128));

	if(angle < angles[0]){
		angle += M_PI*2;
	}
	
	int region = 7;
	for(int i = 1; i < GATE_REGIONS; i++){
		if(angle < angles[i]){
			region = i-1;
			break;
		}
	}
	//Serial.print(angle);
	//Serial.print(',');
	//Serial.println(region);
	
	VectorXf pos(2);
	pos << xState[0],yState[0];
	
	MatrixXf A(2,3);
	A << storedAffineCoeffs[region][0],storedAffineCoeffs[region][1],storedAffineCoeffs[region][2],
			 storedAffineCoeffs[region][3],storedAffineCoeffs[region][4],storedAffineCoeffs[region][5],
	pos = A*pos;
	
	if((xState[1] < 0.0002) && (xState[1] > -0.0002)){
			btn.Ax = (uint8_t) pos[0];
	}
	
	if((yState[1] < 0.0002) && (yState[1] > -0.0002)){
			btn.Ay = (uint8_t) pos[1];
	}
	
	//Serial.print(millis());
	//Serial.print(',');
	//Serial.print(xZ[0],0);
	//Serial.print(',');
	//Serial.print(xState[0],0);
	//Serial.print(',');
	//Serial.print(xState[1]*100000);
	//Serial.print(',');
	//Serial.println(btn.Ax);

	//btn.Ax = (uint8_t) xZ[0];
	//btn.Ay = (uint8_t) yZ[0];
	
	btn.Cx = analogRead(17)>>5;//note it looks like these are swapped from what I thought
	btn.Cy = analogRead(16)>>5;
	
	//Serial.print("C X = ");
	//Serial.println(btn.Cx);
	//Serial.print("C Y = ");
	//Serial.println(btn.Cy);
}
void setPole(){
	for(int i = 0; i < 8; i++){
		//Serial.println();
		//Serial.println("newcommand");
		//Serial.println(btn.arr[i],BIN);
		noInterrupts();
		for(int j = 0; j < 4; j++){
		//Serial.println(btn.arr[i]>>(6-j*2),BIN);
	  int these2bits = (btn.arr[i]>>(6-j*2)) & 3;
		//Serial.print(these2bits,BIN);
	  //Serial.print(these2bits>>1,BIN);
		//Serial.print(these2bits&1,BIN);
			switch(these2bits){
				case 0:
				pollResponse[(i<<2)+j] = 0x08;
				//Serial.print(pollResponse[i+j],HEX);
				//originResponse[i] = 0x08;
				break;
				case 1:
				pollResponse[(i<<2)+j] = 0xE8;
				//Serial.print(pollResponse[i+j],HEX);
				//originResponse[i] = 0xE8;
				break;
				case 2:
				pollResponse[(i<<2)+j] = 0x0F;
				//Serial.print(pollResponse[i+j],HEX);
				//originResponse[i] = 0x0F;
				break;
				case 3:
				pollResponse[(i<<2)+j] = 0xEF;
				//Serial.print(pollResponse[i+j],HEX);
				//originResponse[i] = 0xEF;
				break;
			}
		}
		interrupts();
	}
	 //Serial.println();
	 //delay(1);
}
void communicate(){
	//Serial.println("communicating");
	//clear any commands from the last write
	while(Serial1.available() && (writeQueue > 0)){
		Serial1.read();
		writeQueue --;
	}
	//check if a command has started (bytes available on the serial line)
	if (Serial1.available()){
		//wait for the first 5 bytes of the command to arrive
		while(Serial1.available()<CMD_LENGTH_SHORT){}
		//read those 5 bytes
		for(int i = 0; i <CMD_LENGTH_SHORT; i++){
			cmd[i] = Serial1.read();
		}

		//parse first 4 bytes of the command
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
	//print the command byte over the USB serial connection
	//Serial.println(cmdByte,HEX);
	
	//decide what to do based on the command
	switch(cmdByte){
		case 0x40:
		  while(Serial1.read() != 0xFF){}
		  for(int i = 0; i <POLL_LENGTH; i++){
				Serial1.write(pollResponse[i]);
				//Serial.print(pollResponse[i],HEX);
		  }
			writeQueue = POLL_LENGTH;
		  //Serial.println();
		  //while(Serial1.read() != 0xFF){}
		  break;
		case 0x00:
		  for(int i = 0; i <CMD_LENGTH_LONG; i++){
				Serial1.write(probeResponse[i]);
		  }
			writeQueue = CMD_LENGTH_LONG;
		  //while(Serial1.read() != 0xFF){}
		  break;
		case 0x41:
		  for(int i = 0; i <ORIGIN_LENGTH; i++){
				Serial1.write(originResponse[i]);
		  }
			writeQueue = ORIGIN_LENGTH;
		  //while(Serial1.read() != 0xFF){}
		  break;
		default:
		  //got something strange, try waiting for a stop bit to syncronize
		  while(Serial1.read() != 0xFF){}
			writeQueue = 0;
	  }
	}
}
void calibrate(){
	uint8_t calGuideX[CALIBRATION_POINTS] = {128,228,128,198,128,128,128,58,128,28,128,58,128,128,128,198,128};
	uint8_t calGuideY[CALIBRATION_POINTS] = {128,128,128,198,128,228,128,198,128,128,128,58,128,28,128,58,128};
	if(calStep >= (CALIBRATION_POINTS)){
		Serial.println("the x calibration points are:");
		for(int i = 0; i < CALIBRATION_POINTS; i++){
			Serial.print(calPointsX[i]);
			Serial.print(',');
		}
		Serial.println();
		
		Serial.println("the y calibration points are:");
		for(int i = 0; i < CALIBRATION_POINTS; i++){
			Serial.print(calPointsY[i]);
			Serial.print(',');
		}
		Serial.println();
		
		double x_input[5] = {
		calPointsX[9],
		(calPointsX[7]+calPointsX[11])/2.0,
		(calPointsX[0]+calPointsX[2]+calPointsX[4]+calPointsX[6]+calPointsX[8]+calPointsX[10]+calPointsX[12]+calPointsX[14]+calPointsX[16])/9.0,
		(calPointsX[3]+calPointsX[15])/2.0,
		calPointsX[1]};
		//float x_output[5] = {-100,-70.71,0,70.71,100};
		double x_output[5] = {28,57.29,128,198.71,228};
		
		Serial.println("x inputs are:");
		Serial.print(x_input[0]);
		Serial.print(',');
		Serial.print(x_input[1]);
		Serial.print(',');
		Serial.print(x_input[2]);
		Serial.print(',');
		Serial.print(x_input[3]);
		Serial.print(',');
		Serial.println(x_input[4]);
		
		double y_input[5] = {
		calPointsY[13],
		(calPointsY[11]+calPointsY[15])/2.0,
		(calPointsY[0]+calPointsY[2]+calPointsY[4]+calPointsY[6]+calPointsY[8]+calPointsY[10]+calPointsY[12]+calPointsY[14]+calPointsY[16])/9.0,
		(calPointsY[3]+calPointsY[7])/2.0,
		calPointsY[5]};
		//float y_output[5] = {-100,-70.71,0,70.71,100};
		double y_output[5] = {28,57.29,128,198.71,228};	

		Serial.println("y inputs are:");
		Serial.print(y_input[0]);
		Serial.print(',');
		Serial.print(y_input[1]);
		Serial.print(',');
		Serial.print(y_input[2]);
		Serial.print(',');
		Serial.print(y_input[3]);
		Serial.print(',');
		Serial.println(y_input[4]);		
		
		//fitCurve(FIT_ORDER, 5, x_input, x_output, FIT_ORDER+1, xCoeffs);
		//fitCurve(FIT_ORDER, 5, y_input, y_output, FIT_ORDER+1, yCoeffs);
		double tempXCoeffs[FIT_ORDER+1];
		double tempYCoeffs[FIT_ORDER+1];
		fitCurve(FIT_ORDER, 5, x_input, x_output, FIT_ORDER+1,  tempXCoeffs);
		fitCurve(FIT_ORDER, 5, y_input, y_output, FIT_ORDER+1, tempYCoeffs);
		
		for(int i = 0; i<= FIT_ORDER; i++){
			fitCoeffs[i] = tempXCoeffs[i];
			fitCoeffs[i+FIT_ORDER+1] = tempYCoeffs[i];
		}
		
		float xZeroError = 128 -(fitCoeffs[0]*(x_input[2]*x_input[2]*x_input[2]) + fitCoeffs[1]*(x_input[2]*x_input[2]) + fitCoeffs[2]*x_input[2] + fitCoeffs[3]);
		float yZeroError = 128 -(fitCoeffs[4]*(y_input[2]*y_input[2]*y_input[2]) + fitCoeffs[5]*(y_input[2]*y_input[2]) + fitCoeffs[6]*y_input[2] + fitCoeffs[7]);
		
		//Adjust the fit so that the stick zero position is exactly 128
		fitCoeffs[3] -= xZeroError;
		fitCoeffs[7] -= yZeroError;
		
		float xPointsCleaned[9];
		float yPointsCleaned[9];
		xPointsCleaned[0] = x_input[2];
		yPointsCleaned[0] = y_input[2];
		for(int i = 1; i<9; i++){
			xPointsCleaned[i] = calPointsX[i*2-1];
			yPointsCleaned[i] = calPointsY[i*2-1];
		}

		float xLinearized[9];
		float yLinearized[9];
		
		for(int i = 0; i<9; i++){
			xLinearized[i] = fitCoeffs[0]*(xPointsCleaned[i]*xPointsCleaned[i]*xPointsCleaned[i]) + fitCoeffs[1]*(xPointsCleaned[i]*xPointsCleaned[i]) + fitCoeffs[2]*xPointsCleaned[i] + fitCoeffs[3];
			yLinearized[i] = fitCoeffs[4]*(yPointsCleaned[i]*yPointsCleaned[i]*yPointsCleaned[i]) + fitCoeffs[5]*(yPointsCleaned[i]*yPointsCleaned[i]) + fitCoeffs[6]*yPointsCleaned[i] + fitCoeffs[7];
		}

		float xNotchPoints[9] = {128,228,198.71,128,57.29,28,57.29,128,198.71};
		float yNotchPoints[9] = {128,128,198.71,228,198.71,128,57.29,28,57.29};
		
		
		notchCalibrate(xLinearized,yLinearized,xNotchPoints,yNotchPoints,GATE_REGIONS,storedAffineCoeffs,angles);
/* 		
		MatrixXf xX(5,5);
		MatrixXf yX(5,5);
		VectorXf out(5);
		VectorXf yY(5);
		
		xX << x_input[0]*x_input[0]*x_input[0]*x_input[0],  x_input[0]*x_input[0]*x_input[0],  x_input[0]*x_input[0], x_input[0],1,
				x_input[1]*x_input[1]*x_input[1]*x_input[1],  x_input[1]*x_input[1]*x_input[1],  x_input[1]*x_input[1], x_input[1],1,
        x_input[2]*x_input[2]*x_input[2]*x_input[2],  x_input[2]*x_input[2]*x_input[2],  x_input[2]*x_input[2], x_input[2],1,
				x_input[3]*x_input[3]*x_input[3]*x_input[3],  x_input[3]*x_input[3]*x_input[3],  x_input[3]*x_input[3], x_input[3],1,
        x_input[4]*x_input[4]*x_input[4]*x_input[4],  x_input[4]*x_input[4]*x_input[4],  x_input[4]*x_input[4], x_input[4],1;
				
		yX << y_input[0]*y_input[0]*y_input[0]*y_input[0],  y_input[0]*y_input[0]*y_input[0],  y_input[0]*y_input[0], y_input[0],1,
				y_input[1]*y_input[1]*y_input[1]*y_input[1],  y_input[1]*y_input[1]*y_input[1],  y_input[1]*y_input[1], y_input[1],1,
        y_input[2]*y_input[2]*y_input[2]*y_input[2],  y_input[2]*y_input[2]*y_input[2],  y_input[2]*y_input[2], y_input[2],1,
				y_input[3]*y_input[3]*y_input[3]*y_input[3],  y_input[3]*y_input[3]*y_input[3],  y_input[3]*y_input[3], y_input[3],1,
        y_input[4]*y_input[4]*y_input[4]*y_input[4],  y_input[4]*y_input[4]*y_input[4],  y_input[4]*y_input[4], y_input[4],1;
				
		out << 28,57.29,128,198.71,228;
		
		VectorXf xCoeffs(5);
		VectorXf yCoeffs(5);
		xCoeffs = xX.colPivHouseholderQr().solve(out);
		yCoeffs = yX.colPivHouseholderQr().solve(out);
		 */
		 
		Serial.println("x,y coefficients are:");
		for(int i = 0; i <= FIT_ORDER; i++){
			//fitCoeffs[i] = xCoeffs[i];
			//fitCoeffs[i+FIT_ORDER+1] = yCoeffs[i];
			Serial.print(fitCoeffs[i],12);
			Serial.print(',');
			Serial.println(fitCoeffs[i+FIT_ORDER+1],12);
		}

		//EEPROM.put(FIT_ADDRESS,fitCoeffs);
		
		//first (FIT_ORDER+1)*2*4 bytes are the fit coefficients
		EEPROM.put( 0, fitCoeffs );
		//next GATE_REGIONS*6*4 bytes are the affine transformation coefficients
		EEPROM.put( (FIT_ORDER+1)*2*4, storedAffineCoeffs );
		//and last is the angles for the different gate regions
		EEPROM.put( (FIT_ORDER+1)*2*4+GATE_REGIONS*6*4, angles );
		
		calStep = -1;
	}
	else{
		//calPointsX[calStep] = ((float)AStickHX)/8192.0;
		//calPointsY[calStep] = ((float)AStickHY)/8192.0;
		calPointsX[calStep] = ((float)AStickHX);
		calPointsY[calStep] = ((float)AStickHY);
		
		btn.Cx = calGuideX[calStep];
		btn.Cy = calGuideY[calStep];
	}

}
void runKalman(VectorXf& xZ,VectorXf& yZ){
	//Serial.println("Running Kalman");
	
	unsigned int thisMicros = micros();
	unsigned int dT = thisMicros-lastMicros;
	lastMicros = thisMicros;
	//Serial.print("the loop time is: ");
	//Serial.println(dT);
	
	
	//MatrixXf tryF(2);
	Fmat << 1,dT-xDamping/2*dT*dT,
			 0,1-xDamping*dT;

	xQ << (dT*dT*dT*dT>>2), (dT*dT*dT>>1),
			 (dT*dT*dT>>1), (dT*dT);
	yQ = xQ * yAccelVar;
	xQ = xQ * xAccelVar;
	
	MatrixXf sharedH(1,2);
	sharedH << 1,0;
	
	//Serial.println('H');
	//print_mtxf(H)
	
	MatrixXf xR(1,1);
	//MatrixXf yR(1,1);
	float offset_squared = (xZ[0]-128)*(xZ[0]-128)+(yZ[0]-128)*(yZ[0]-128);
	xR << X_ADC_VAR_PEAK/(offset_squared/X_ADC_VAR_SPREAD+1);
	
	//dT = micros()-lastMicros;
	//Serial.println(dT);
	//print_mtxf(xP);
	kPredict(xState,Fmat,xP,xQ);
	//dT = micros()-lastMicros;
	//Serial.println(dT);
	
	kPredict(yState,Fmat,yP,yQ);
	//dT = micros()-lastMicros;
	//Serial.println(dT);
	
	kUpdate(xState,xZ,xP,sharedH,xR);
	//dT = micros()-lastMicros;
	//Serial.println(dT);
	
	kUpdate(yState,yZ,yP,sharedH,xR);
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
	int sizeMeas = Z.size();
	MatrixXf A(1,2);
	A = P*H.transpose();
	MatrixXf B(1,1);
	B = H*A+R;
	MatrixXf K(2,1);
	K = A*B.inverse();
	//print_mtxf(K);
	//K = MatrixXf::Identity(sizeState,sizeState);
	MatrixXf C(1,1); 
	C = Z - H*X;
	X = X + K*(Z - H*X);
	//X = X + K*(measX-X[0]);
	
	MatrixXf D = MatrixXf::Identity(sizeState,sizeState) - K*H;
	P = D*P*D.transpose() + K*R*K.transpose();
	
	//print_mtxf(P);
}

void notchCalibrate(float* xIn, float* yIn, float* xOut, float* yOut, int regions, float allAffineCoeffs[][6], float regionAngles[]){
	for(int i = 1; i < regions; i++){
/* 		MatrixXf system(6,6);
		system << xIn[0],yIn[0],0,0,1,0,
				 0,0,xIn[0],yIn[0],0,1,
				 xIn[i],yIn[i],0,0,1,0,
				 0,0,xIn[i],yIn[i],0,1,
				 xIn[i+1],yIn[i+1],0,0,1,0,
				 0,0,xIn[i+1],yIn[i+1],0,1;
		
		Serial.println("The system to solve is:");
		print_mtxf(system);
		
		VectorXf transformed(6);
		transformed << xOut[0],yOut[0],xOut[i],yOut[i],xOut[i+1],yOut[i+1];
		
		Serial.println("The desired points (after transformation) are:");
		print_mtxf(transformed);
		
		VectorXf affineCoeffs(6);
		affineCoeffs = system.colPivHouseholderQr().solve(transformed);
		
		Serial.println("The vector with the solved coefficients is:");
		print_mtxf(affineCoeffs);
		
		Serial.println("The affince transformation coordinates are:");
		for(int j = 0; j <6;j++){
			allAffineCoeffs[i-1][j] = affineCoeffs[j];
			Serial.print(allAffineCoeffs[i-1][j]);
			Serial.print(",");
		} */
		MatrixXf pointsIn(3,3);
		pointsIn << xIn[0],xIn[i],xIn[i+1],
								yIn[0],yIn[i],yIn[i+1],
								1,1,1;
		MatrixXf pointsOut(3,3);
		pointsOut << xOut[0],xOut[i],xOut[i+1],
								 yOut[0],yOut[i],yOut[i+1],
								 1,1,1;
		
		MatrixXf A(3,3);
		A = pointsOut*pointsIn.inverse();
		
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
void print_mtxf(const Eigen::MatrixXf& X)  
{
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
