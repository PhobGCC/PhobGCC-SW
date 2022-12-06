#ifndef COMMS_H
#define COMMS_H

//This must be included at the end of the Teensy board header file

#include "TeensyTimerTool.h"
#include "../common/structsAndEnums.h"
#include "../common/variables.h"

TeensyTimerTool::OneShotTimer timer1;

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
};


#ifdef TEENSY4_0
#ifdef HALFDUPLEX
void resetSerial(){
#ifdef ENABLE_LED
	digitalWriteFast(_pinLED,!digitalReadFast(_pinLED));
#endif //ENABLE_LED
	Serial2.clear();
	Serial2.flush();
	Serial2.begin(_slowBaud,SERIAL_HALF_DUPLEX);
	digitalWriteFast(_pinLED,LOW);
};
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
};
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
};
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
};
#endif //HALFDUPLEX

#endif // TEENSY4_0


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
};
void setSerialFast(){
	UART1_BDH = _fastBDH;
	UART1_BDL = _fastBDL;
	UART1_C4 = _fastC4;
	UART1_C2 &= ~UART_C2_RE;
};
void resetSerial(){
	UART1_BDH = _slowBDH;
	UART1_BDL = _slowBDL;
	UART1_C4 = _slowC4;
	UART1_C2 |= UART_C2_RE;
	Serial2.clear();
};
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
};
#endif // TEENSY3_2


void commsSetup(Buttons &btn) {
	setCommResponse(_originResponse, btn);

	//set up communication interrupts, serial, and timers
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
};


#endif // COMMS_H
