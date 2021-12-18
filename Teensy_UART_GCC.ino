//uses a teensy 3.2, hardware serial pins 0 (RX) and 1(TX), diode from 0 to 1 to make the output half duplex, data line connected to pin 0(RX)

#define CMD_LENGTH_SHORT 5
#define CMD_LENGTH_LONG 13
#define ORIGIN_LENGTH 41
#define POLL_LENGTH 33

int cmd[CMD_LENGTH_LONG];

static uint8_t probeResponse[CMD_LENGTH_LONG] = 
{0x08,0x08,0x0F,0xE8,
 0x08,0x08,0x08,0x08,
 0x08,0x08,0x08,0xEF,
 0xFF};
 
static uint8_t pollResponse[POLL_LENGTH] =
{0x08,0x08,0x08,0x08,
 0x08,0x08,0x08,0x08,
 0x08,0x08,0xEF,0xEF,
 0x08,0x08,0xEF,0xEF,
 0x08,0x08,0xEF,0xEF,
 0x08,0x08,0xEF,0xEF,
 0x08,0x08,0xEF,0xEF,
 0x08,0x08,0xEF,0xEF,
 0xFF};
 
static uint8_t originResponse[ORIGIN_LENGTH] =
{0x08,0x08,0x08,0x08,
 0x08,0x08,0x08,0x08,
 0x08,0x08,0xEF,0xEF,
 0x08,0x08,0xEF,0xEF,
 0x08,0x08,0xEF,0xEF,
 0x08,0x08,0xEF,0xEF,
 0x08,0x08,0xEF,0xEF,
 0x08,0x08,0x08,0x08,
 0x08,0x08,0x08,0x08,
 0x08,0x08,0x08,0x08,
 0xFF};

uint8_t cmdByte;

void setup() {
  //start USB serial
  Serial.begin(57600);
  Serial.println("starting");
  //start hardware serail
  Serial1.begin(1000000);
}

void loop() {
  //check if a command has started (bytes available on the serial line)
  if (Serial1.available())
  {
    //wait for the first 5 bytes of the command to arrive
    while(Serial1.available()<CMD_LENGTH_SHORT)
    {
    }
    //read those 5 bytes
    for(int i = 0; i <CMD_LENGTH_SHORT; i++)
    {
      cmd[i] = Serial1.read();
    }

    //parse first 4 bytes of the command
    for(int i = 0; i <CMD_LENGTH_SHORT-1; i++)
    {
      switch(cmd[i])
      {
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
    switch(cmdByte)
      {
        case 0x40:
          while(Serial1.read() != 0xFF)
          {
          }
          for(int i = 0; i <POLL_LENGTH; i++)
          {
            Serial1.write(pollResponse[i]);
          }
          while(Serial1.read() != 0xFF)
          {
          }
          break;
        case 0x00:
          for(int i = 0; i <CMD_LENGTH_LONG; i++)
          {
            Serial1.write(probeResponse[i]);
          }
          while(Serial1.read() != 0xFF)
          {
          }
          break;
        case 0x41:
          for(int i = 0; i <ORIGIN_LENGTH; i++)
          {
            Serial1.write(originResponse[i]);
          }
          while(Serial1.read() != 0xFF)
          {
          }
          break;
        default:
          //got something strange, try waiting for a stop bit to syncronize
          while(Serial1.read() != 0xFF)
          {
          }
      }
  }
  //reset the command
  for(int i = 0; i <CMD_LENGTH_LONG; i++)
    {
      cmd[i] = -1;
    }
}
