//*****************************************************************************
// PIXY.c
// This code uses the HS12 SPI port to communicate with the CMUCAM5 "Pixy"
//   camera.  The code was taken from the Pixy porting guide which provides 
//   code written in a generic C-language format.  There is additional code
//   available written for the Arduino (e.g. from Adafruit pixy pet project)
//   which can be used as well.
// This file must be placed above your main.c file so that its initialized 
//  varilables are available to main
// Porting guide for the pixy cam 
//   http://www.cmucam.org/projects/cmucam5/wiki/Porting_Guide
//*****************************************************************************
  
#include <hidef.h>      /* common defines and macros */
#include "derivative.h"      /* derivative-specific definitions */

#include <stdlib.h>
#include <stdio.h>
#include "PixyHS12.h"
#include "PLL.H"
#include "SCI0_Buffered.h"

static	int32_t pan_m_proportionalGain = 200;
static	int32_t pan_m_derivativeGain = 100;
static	int32_t tilt_m_proportionalGain = 200;
static	int32_t tilt_m_derivativeGain = 100;

extern void panUpdate(int32_t);
extern int32_t pan_m_pos, tilt_m_pos;




//These variables are declared in PixyHS12.h and defined here.
BlockType g_blockType;
Block *g_blocks;
uint8_t g_outBuf[PIXY_OUTBUF_SIZE];
uint8_t g_outLen = 0;
uint8_t g_outWriteIndex = 0;
uint8_t g_outReadIndex = 0;

// variables for Block and Frame Parsing in Pixy.c
int g_skipStart = 0;

//**** getByte
// SPI routine for HS12 to read one byte of data and send one byte of data through SPI0., 
// To be able to read data you need to send data first

uint8_t getByte(uint8_t out){
  unsigned char value;
  while(!SPI0SR_SPTEF);
  // while(((SPI1SR&0x20)==0)){}; // wait for data register to empty
  SPI0DR=out; //write data to be sent in SPI0DR
  while(!SPI0SR_SPIF);
  // while(((SPI1SR&0x80)==0)){}; //wait for slave data to be in data register
  value=SPI0DR; //read data received in SPI0DR
  return value; 
}

//**** getWord
uint16_t getWord()
{
  // ordering is big endian because Pixy is sending 16 bits through SPI
  // for other protocols (I2C, UART, the byte ordering would be little endian 
  uint16_t w;
  uint8_t c, cout = 0;

  if (g_outLen)    // If there is data in the queue we send a data sync byte
  {                // and then send a data byte... until the queue is empty
    w = getByte(PIXY_SYNC_BYTE_DATA);
    cout = g_outBuf[g_outReadIndex++];   // read byte from the buffer and
                                         // increment the pointer
    g_outLen--;   // decrement the buffer char count after reading a byte
    if (g_outReadIndex==PIXY_OUTBUF_SIZE) // if we are at the end of the queue
      g_outReadIndex = 0;                 // then we circle back to the top
  }
  else {
    // there is no data to send so we use the standard sync byte and
    // follow with a 0 byte (cout is initialized to 0)
    w = getByte(PIXY_SYNC_BYTE); // send out sync byte
  }
  w = w << 8;  // left shift by 8-bits to move to MSB position
  c = getByte(cout); // send out data byte
  w = w | c; //|= c;   // compose the 16 bit word we just read.
  return w;
}

//**** send
// "send" accepts a buffer of data (as a pointer) and writes it to the circular
// queue so that byte by byte it is sent as we read the pixy block data
int send(uint8_t *data, int len)
{
  int i;

  // check to see if we have enough space in our circular queue
  if (g_outLen+len>PIXY_OUTBUF_SIZE)
    return -1;

  g_outLen += len;
  for (i=0; i<len; i++)
  {
    g_outBuf[g_outWriteIndex++] = data[i];
    if (g_outWriteIndex==PIXY_OUTBUF_SIZE)
      g_outWriteIndex = 0;
  }
  return len;
}




//**** getStart
// Looks for the two sync bytes that indicate the start of a frame which could contain
// multiple blocks
int getStart(void)
{
  uint16_t w, lastw;

  lastw = 0xffff;

  while(1)
  {
    w = getWord();
    if (w==0 && lastw==0)
      return 0; // no start code  
    else if (w==PIXY_START_WORD && lastw==PIXY_START_WORD)
    {
      g_blockType = NORMAL_BLOCK;
      return 1; // code found!
    }
    else if (w==PIXY_START_WORD_CC && lastw==PIXY_START_WORD)
    {
      g_blockType = CC_BLOCK; // found color code block
      return 1;
    }    
    else if (w==PIXY_START_WORDX) 
      getByte(0); // we're out of sync! (backwards)

    lastw = w; 
  }
}

uint16_t getBlocks(uint16_t maxBlocks)
{
  uint8_t i;
  uint16_t w, blockCount, checksum, sum;
  Block *block;

  // On first run g_skipStart=0 so we will enter the if statement calling getStart()
  // getStart will look for the double syncwords associated with the start of a frame
  // If there is no data, we return from the function with a 0 value.  Otherwise we return
  // with with a flag value of 1 indicating a successful frame identification.
  // If on the otherhand we find g_skipStart = 1, this means that in the block reading 
  // process below we have already identified the start of another frame (we read two 
  // back-to-back sync words).  Thus the next word we read will be the checksum so we do not
  // want to call getStart... we just want to continue with the block read process.  In this
  // case we reset g_skipStart to 0 so it can be re-set to 1 as necessary.  
  
  // Consider the following diagram of sorts which lists a frame start, three blocks (for three
  //   objects) and then the start of another frame.  The following text explains the operation
  //   of the getBlocks() main loop...
  /*
   * ** start of frame begins with an extra sync
   * sync           -- getStart reads these two sync bytes so that next word is the checksum
   * block_sync
   * 
   * block_data     -- the blockCount for loop reads a word and checks to see if it is a sync word.
   *                -- If getStart has been called, the first word will be the checksum and we fall
   *                -- through the first if and proceed to read the block contents calculating and
   *                -- checking the checksum as we go.  
   *
   * block_sync     -- at the end of the getBlocks function, we read another word which will normally be
   * block_data     -- the next block_sync word and thus the same process described above continues until...
   *
   * block_sync
   * block_data
   *   
   * ** start of next frame  
   * sync          -- We reach the start of a new frame.  We read a sync word and loop back in the for loop
   * block_sync    -- but, this time when we read the next word it is again a sync word so we set the
   * block_data    -- g_skipStart to 1 (since at this point we have already read the two sync words) and
   *               -- we return from the function call with total block count read.
  */
  
  if (!g_skipStart)     
  {
    if (getStart()==0)
      return 0;
  }
  else
    g_skipStart = 0;

  for(blockCount=0; blockCount<maxBlocks && blockCount<PIXY_ARRAYSIZE;)
  {
    checksum = getWord();
    if (checksum==PIXY_START_WORD) // we've reached the beginning of the next frame
    {
      g_skipStart = 1;
      g_blockType = NORMAL_BLOCK;
      return blockCount;
    }
    else if (checksum==PIXY_START_WORD_CC)
    {
      g_skipStart = 1;
      g_blockType = CC_BLOCK;
      return blockCount;
    }
    else if (checksum==0)
      return blockCount;

    block = g_blocks + blockCount;

    for (i=0, sum=0; i<sizeof(Block)/sizeof(uint16_t); i++)
    {
      if (g_blockType==NORMAL_BLOCK && i>=5) // no angle for normal block
      {
        block->angle = 0;
        break;
      }
      w = getWord();
      sum += w;
      *((uint16_t *)block + i) = w;
    }

    // check checksum
    if (checksum==sum){
      blockCount++; 
    }
    else
 //     printf("checksum error!\n");

    w = getWord();
    if (w==PIXY_START_WORD)
      g_blockType = NORMAL_BLOCK;
    else if (w==PIXY_START_WORD_CC)
      g_blockType = CC_BLOCK;
    else
      return blockCount;
  }
}

//**** setServos
// fill a siz element buffer with the sync and servo data ordered as Little Endian
// This array is then passed to "send" which places it in the output queue
int setServos(uint16_t s0, uint16_t s1)
{
  uint8_t outBuf[6];
  uint8_t low_byte = 0;
  uint8_t high_byte = 0;

  outBuf[0] = 0x00;
  outBuf[1] = PIXY_SERVO_SYNC;   
  high_byte = (uint8_t)
  (s0>>8);
  low_byte  = (uint8_t)(s0 & 0xff);
  outBuf[2] = low_byte;       // output as little endian
  outBuf[3] = high_byte;     

  high_byte = (unsigned char)(s1>>8);
  low_byte  = (unsigned char)(s1 & 0xff);
  outBuf[4] = low_byte;       // output as little endian
  outBuf[5] = high_byte;

// This method does not work because HS12 is BigEndian
//  *(uint16_t *)(outBuf + 2) = s0;
//  *(uint16_t *)(outBuf + 4) = s1;

  return send(outBuf, 6);
}

//**** setBrightness
int setBrightness(uint8_t brightness)
{
  uint8_t outBuf[3];

  outBuf[0] = 0x00;
  outBuf[1] = PIXY_CAM_BRIGHTNESS_SYNC; 
  outBuf[2] = brightness;

  return send(outBuf, 3);
}

//**** setLED
int setLED(uint8_t r, uint8_t g, uint8_t b)
{
  uint8_t outBuf[5];

  outBuf[0] = 0x00;
  outBuf[1] = PIXY_LED_SYNC; 
  outBuf[2] = r;
  outBuf[3] = g;
  outBuf[4] = b;

  return send(outBuf, 5);
}

//**** initSPI
// The following function initializes the SPI0 interface for CMUCAM5    

void initSPI(void){  //SCI0 initialization routine
  DDRS=DDRS|0x60; //0b01100000; //ps6,ps5 outputs,ps4 input, Ps4:MISO, Ps5:MOSI, Ps6:SCK, 
  //  We are not using the Slave select pin SS so leave pin at default.  	
  //  For SCI1 use portH for pins -- See the DUG manual for pinouts  
  //  DDRH=DDRH|0x06; //0b00000110; //ph2,ph1 outputs,ph0 input, PH0:MISO, PH1:MOSI, PH2:SCK, 
  //	To force slave select high you could execute PTH |= 0x08; and then enable pullup on the pin
	SPI0CR1_LSBFE = 0;	// MSB sent first                  
	SPI0CR1_SSOE = 0; 	// Slave select not used                   
	SPI0CR1_CPHA = 0;	  // First edge is used for Latching into Slave (pixy) see HS12 man and Valvano Fig 8.8                   
	SPI0CR1_CPOL = 0;	  // Clock idles low for cmucam so use Active High Idle low SCK                   
	SPI0CR1_MSTR = 1;	  // HS12 is master                   
	SPI0CR1_SPTIE = 0;	// transmit buffer empty interrupt disabled for now (SPTEF flag)                 
	SPI0CR1_SPE = 1;	  // SPI system enabled                    
	SPI0CR1_SPIE = 0;	  // 8-SCK cycles complete interrupt disabled for now
  //	SPI0CR1 = 0x50;
                      // Equivalent to SPI0CR1 = 0x50;
	SPI0CR2_SPC0 = 0;	  // not using single pin bidirectional mode so accept serial control pin default
	SPI0CR2_SPISWAI=0;	// do not stop in wait mode (power saving)
	SPI0CR2_BIDIROE=0;	// not using bidirectional mode
	SPI0CR2_MODFEN=0;	  // we are not using slave select for we do not use Mode Fault monitoring
	    					      // 	 This is used when there could be multiple masters...
  // Equivalent to SPI1CR2=0x00;
  //  SPI0CR2=0x00;
  // Assume that we are using the PLL and a 24MHz clock.  Then we want a divisor of 24 for the busclock
  //   to obtain the SPI clock of 1 Mhz.  Note the Pixy can operate faster but the cables are
  //	 not shielded and noise could arise... 
  // SPI0BR settings
  // BaudRateDivisor = (SPPR+1) x 2^(SPR+1) 
  // 24Mhz/1Mhz = 24 = 6*4.  Set SPPR2~SPPR0 to 101 (5) and 
  // SPR2~SPR0 to 001 (1).  Thus write $51 to the SPI0BR register.  
  // Alternatively you can use $22 (3*8) to get a divisor of 24. 
  // Note if PLL is not used and ECLK = 8Mhz then set SPIBR to 0x02 for divide by 8
	SPI0BR=0x02; 
  return;   
}



//**** init
// Allocate ram to hold the block arrays
void init()
{
  g_blocks = (Block *)malloc((sizeof(Block)*PIXY_ARRAYSIZE));
}

// ServoLoop Update 
// Calculates new output based on the measured
// error and the current state.
// Static in this context means that these variables are only visible
// to this main function file.
static int32_t pan_m_pos = RCS_CENTER_POS;
static	int32_t pan_m_prevError = 0x80000000;

void panUpdate(int32_t error)
{
	int32_t velocity;
//	char buf[32];
	if (pan_m_prevError!=0x80000000)
	{	
		velocity = (error*pan_m_proportionalGain + (error - pan_m_prevError)*pan_m_derivativeGain)>>10;
    /*if (debug2){
      printf("pan_pos = %ld, velocity = %ld \n",pan_m_pos,velocity);
    } */
		pan_m_pos += velocity;
		if (pan_m_pos>RCS_MAX_POS) 
		{
			pan_m_pos = RCS_MAX_POS;     // set maximum pan position
		}
		else if (pan_m_pos<RCS_MIN_POS) 
		{
			pan_m_pos = RCS_MIN_POS;     // set mimimum pan position
		}
	}
	pan_m_prevError = error;
}                    
                         
extern int BlocksInFrame;

void FollowBlock(int TrackedNum){

    //putcharSCI1(g_blocks[TrackedNum].signature);
    //putcharSCI1(g_blocks[TrackedNum].x);
      
      
     BlocksInFrame=getBlocks(100);   
     TrackBlock(BlocksInFrame);
     
     if (pan_m_pos > 0x220){         //move left
      
      //SetGait(0x01);
      TurnLeft();
      //Delay(50);
        
      }
      if ( (pan_m_pos > 0x180) && (pan_m_pos < 0x220) ){    //go straight
        
      //SetGait(0x01);
     StraifForward();
     // Delay(50);  //
      } 
      if (pan_m_pos < 0x180){         //move right
          
      // SetGait(0x01);
       TurnRight();
     // Delay(50);  
      }
      /* if (pan_m_pos > 0x220)
      {         //move left
      //SetGait(0x02);//this algrithem is wrong, I can not send two movement at same time
      //TurnLeft();
      
         if((angel > 3900) && (angel <= 800)){  //Front
         if (length < 30)
           StraifLeft();// Stop();
          else
            TurnLeft();//StraifForward();
        } else if((angel > 2800) && (angel <= 3900)){    //Left
          if (length < 30)
            StraifForward();//Stop();
          else
            TurnLeft();//StraifLeft();
        } else if((angel > 2100) && (angel <= 2800)){    //Back
          if (length < 30)
            StraifLeft();//Stop();
          else
           TurnLeft();// StraifReverse();
        } else if((angel > 800) && (angel <= 2100)){     //Right
          if (length < 30)
            StraifLeft();//Stop();
          else
            TurnLeft();//StraifRight();
        }
      }
      if ( (pan_m_pos > 0x180) && (pan_m_pos < 0x220) ){    //go straight
      //SetGait(0x02);
     // StraifForward();
   if((angel > 3900) && (angel <= 800)){  //Front
         if (length < 30)
            Stop();
         else
            StraifForward();
        }
      } 
      if (pan_m_pos < 0x180){         //move right   
      // SetGait(0x02);
      //TurnRight();
       if((angel > 3900) && (angel <= 800)){  //Front
         if (length < 30)
            StraifRight();//Stop();
          else
            TurnRight();//StraifForward();
        } else if((angel > 2800) && (angel <= 3900)){    //Left
          if (length < 30)
            StraifRight();//Stop();
          else
            TurnRight();//StraifLeft();
        } else if((angel > 2100) && (angel <= 2800)){    //Back
          if (length < 30)
           StraifRight();// Stop();
          else
            TurnRight();//StraifReverse();
        } else if((angel > 800) && (angel <= 2100)){     //Right
          if (length < 30)
           StraifForward(); //Stop();
          else
            TurnRight();//StraifRight();
        }
      }
          
    }*/
 
}


//-----------------same----------------------
static	int32_t tilt_m_pos = RCS_CENTER_POS;
static	int32_t tilt_m_prevError = 0x80000000;

void tiltUpdate(int32_t error)
{
	int32_t velocity;
//	char buf[32];
	if (tilt_m_prevError!=0x80000000)
	{	
		velocity = (error*tilt_m_proportionalGain + (error - tilt_m_prevError)*tilt_m_derivativeGain)>>10;
		
    /*if (debug2){
      printf("tilt_pos = %ld, velocity = %ld \n",tilt_m_pos,velocity);
    } */

		tilt_m_pos += velocity;
		if (tilt_m_pos>RCS_MAX_POS) 
		{
			tilt_m_pos = RCS_MAX_POS; 
		}
		else if (tilt_m_pos<RCS_MIN_POS) 
		{
			tilt_m_pos = RCS_MIN_POS;
		}
	}
	tilt_m_prevError = error;
}
static int oldX, oldY, oldSignature;

//---------------------------------------
// Track blocks via the Pixy pan/tilt mech
// (based in part on Pixy CMUcam5 pantilt example)
//-----------------same----------------------
int TrackBlock(int blockCount)
{
	int trackedBlock = 0;
	long maxSize = 0;
	int32_t panError, tiltError;
	int i,xp,yp;

	for (i = 0; i < blockCount; i++)
	{
	
		if ((oldSignature == 0) || (g_blocks[i].signature == oldSignature))
		{
			long newSize = g_blocks[i].height * g_blocks[i].width;
			if (newSize > maxSize)
			{
				trackedBlock = i;
				maxSize = newSize;
			}
		}

	
    xp = g_blocks[trackedBlock].x;
    yp = g_blocks[trackedBlock].y;
    panError = X_CENTER - g_blocks[trackedBlock].x;
    tiltError = g_blocks[trackedBlock].y - Y_CENTER;

  	panUpdate(panError);
  	tiltUpdate(tiltError);
  	
  }

  setServos(pan_m_pos, tilt_m_pos);

	oldX = g_blocks[trackedBlock].x;
	oldY = g_blocks[trackedBlock].y;
	oldSignature = g_blocks[trackedBlock].signature;

	return trackedBlock;
}
