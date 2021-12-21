//uses a teensy 3.2, hardware serial pins 0 (RX) and 1(TX), diode from 0 to 1 to make the output half duplex, data line connected to pin 0(RX)
//read this for a description of how this works:

#include <math.h>
#include <curveFitting.h>

#define CMD_LENGTH_SHORT 5
#define CMD_LENGTH_LONG 13
#define ORIGIN_LENGTH 41
#define POLL_LENGTH 33
#define CALIBRATION_POINTS 17

#define HALX 21
#define HALY 20

#define FIT_ORDER 3

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
/* 	// byte 0
	uint8_t errS : 1;
	uint8_t errL : 1;
	uint8_t orig : 1;
	uint8_t S : 1;
	uint8_t Y : 1;
	uint8_t X : 1;
	uint8_t B : 1;
	uint8_t A : 1;

	// byte 1
	uint8_t high : 1;
	uint8_t L : 1;
	uint8_t R : 1;
	uint8_t Z : 1;
	uint8_t Du : 1;
	uint8_t Dd : 1;
	uint8_t Dr : 1;
	uint8_t Dl : 1; */

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

unsigned int AStickHX;
unsigned int AStickHY;
double xCoeffs[FIT_ORDER+1] = {0,0,1,0};
double yCoeffs[FIT_ORDER+1] = {0,0,1,0};
int calStep;
unsigned int startBtnSince;
int lastStartBtn;
double calPointsX[CALIBRATION_POINTS];
double calPointsY[CALIBRATION_POINTS];

static uint8_t probeResponse[CMD_LENGTH_LONG] = {
	0x08,0x08,0x0F,0xE8,
	0x08,0x08,0x08,0x08,
	0x08,0x08,0x08,0xEF,
	0xFF};
static uint8_t pollResponse[POLL_LENGTH] = {
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
	btn.errS = 0;
	btn.errL = 0;
	btn.orig = 0;
	btn.high = 1;
	startBtnSince = millis();
	lastStartBtn = 0;
	analogReadResolution(13);
	setPinModes();
	//start USB serial
	Serial.begin(57600);
	//Serial.begin(9600);
	Serial.println("starting");
	//start hardware serail
	Serial1.begin(1000000);
}

void loop() {
	communicate();
	readButtons();
	readSticks();
	setPole();
	if(!calStep){
		calibrate();
	}
}

void setPinModes(){
	pinMode(9,INPUT_PULLUP);
	pinMode(7,INPUT_PULLUP);
	pinMode(10,INPUT_PULLUP);
	pinMode(12,INPUT_PULLUP);
	pinMode(11,INPUT_PULLUP);
	pinMode(6,INPUT_PULLUP);
	pinMode(14,INPUT_PULLUP);
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
	btn.L = !digitalRead(14); //note, need to cut and reroute this trace, was to pin 1
	btn.R = !digitalRead(8);
	btn.Du = !digitalRead(3);
	btn.Dd = !digitalRead(4);
	btn.Dl = !digitalRead(2);
	btn.Dr = !digitalRead(5);
	
	if(btn.S && !lastStartBtn){
		startBtnSince = millis();
	}
	if(startBtnSince > 5000){
		calStep = 0;
	}
}
void readSticks(){
	//btn.Ax = analogRead(18);
	//btn.Ay = analogRead(19);
	btn.La = analogRead(22)>>5;
	btn.Ra = analogRead(15)>>5;
	
	AStickHX = (float)analogRead(HALX);
	AStickHY = (float)analogRead(HALY);
	btn.Ax = (uint8_t) (xCoeffs[0]*pow(AStickHX,3) + xCoeffs[1]*pow(AStickHX,2) + xCoeffs[2]*pow(AStickHX,1) + xCoeffs[2]*AStickHX) + 128;
	btn.Ay = (uint8_t) (yCoeffs[0]*pow(AStickHY,3) + yCoeffs[1]*pow(AStickHY,2) + yCoeffs[2]*pow(AStickHY,1) + yCoeffs[2]*AStickHY) + 128;
	
	btn.Cx = analogRead(17)>>5;//note it looks like these are swapped from what I thought
	btn.Cy = analogRead(16)>>5;
}
void setPole(){
	for(int i = 0; i < 8; i++){
		//Serial.println();
		//Serial.println("newcommand");
		//Serial.println(btn.arr[i],BIN);
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
				originResponse[i] = 0x08;
				break;
				case 1:
				pollResponse[(i<<2)+j] = 0xE8;
				//Serial.print(pollResponse[i+j],HEX);
				originResponse[i] = 0xE8;
				break;
				case 2:
				pollResponse[(i<<2)+j] = 0x0F;
				//Serial.print(pollResponse[i+j],HEX);
				originResponse[i] = 0x0F;
				break;
				case 3:
				pollResponse[(i<<2)+j] = 0xEF;
				//Serial.print(pollResponse[i+j],HEX);
				originResponse[i] = 0xEF;
				break;
			}
		}
	}
	 //Serial.println();
	 //delay(1);
}
void communicate(){
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
	Serial.println(cmdByte,HEX);
	
	//decide what to do based on the command
	switch(cmdByte){
		case 0x40:
		  while(Serial1.read() != 0xFF){}
		  for(int i = 0; i <POLL_LENGTH; i++){
				Serial1.write(pollResponse[i]);
				Serial.print(pollResponse[i],HEX);
		  }
		  Serial.println();
		  while(Serial1.read() != 0xFF){}
		  break;
		case 0x00:
		  for(int i = 0; i <CMD_LENGTH_LONG; i++){
				Serial1.write(probeResponse[i]);
		  }
		  while(Serial1.read() != 0xFF){}
		  break;
		case 0x41:
		  for(int i = 0; i <ORIGIN_LENGTH; i++){
				Serial1.write(originResponse[i]);
		  }
		  while(Serial1.read() != 0xFF){}
		  break;
		default:
		  //got something strange, try waiting for a stop bit to syncronize
		  while(Serial1.read() != 0xFF){}
	  }
	}
}
void calibrate(){
	if(btn.A){
		calPointsX[calStep] = AStickHX;
		calPointsY[calStep] = AStickHY;
		calStep++;
	}
	if(calStep >= CALIBRATION_POINTS){
		double x_input[5] = {
		calPointsX[9],
		(calPointsX[7]+calPointsX[11])/2.0,
		(calPointsX[0]+calPointsX[2]+calPointsX[4]+calPointsX[6]+calPointsX[8]+calPointsX[10]+calPointsX[12]+calPointsX[14]+calPointsX[16])/8.0,
		(calPointsX[3]+calPointsX[15])/2.0,
		calPointsX[1]};
		double x_output[5] = {-100,-70.71,0,70.71,100};
		
		double y_input[5] = {
		calPointsY[13],
		(calPointsY[11]+calPointsY[15])/2.0,
		(calPointsY[0]+calPointsY[2]+calPointsY[4]+calPointsY[6]+calPointsY[8]+calPointsY[10]+calPointsY[12]+calPointsY[14]+calPointsY[16])/9.0,
		(calPointsY[3]+calPointsY[7])/2.0,
		calPointsY[4]};
		double y_output[5] = {-100,-70.71,0,70.71,100};		
		
		fitCurve(FIT_ORDER, 5, x_input, x_output, FIT_ORDER+1, xCoeffs);
		fitCurve(FIT_ORDER, 5, y_input, y_output, FIT_ORDER+1, yCoeffs);
	}
	calStep = -1;
}
