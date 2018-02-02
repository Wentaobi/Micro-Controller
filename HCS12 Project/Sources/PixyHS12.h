/* PixyHS12.h
This is optimized for CodeWarrior HCS12 5.0
 
Note, for variables shared across compilation units (c-files), you should declare them in a header file 
with the extern keyword, then define them in a single source file, without the extern keyword. 
The single source file should be the one sharing the header file's name, for best practice. 

Author: Mark Paulik, 2015.Dec 3
*/

#ifndef _PixyHS12_h
#define _PixyHS12_h
typedef unsigned char uint8_t; /*unsigned 8 bit definition */
typedef unsigned int uint16_t; /*unsigned 16 bit definition*/
typedef unsigned long uint32_t; /*unsigned 32 bit definition*/
typedef signed long int32_t; // signed 32 bit definition

//datatypes
typedef enum 
{
  NORMAL_BLOCK,
  CC_BLOCK // color code block
} BlockType;

typedef struct  
{
  uint16_t signature; 
  uint16_t x;
  uint16_t y;
  uint16_t width;
  uint16_t height;
  uint16_t angle; // angle is only available for color coded blocks
} Block;

// typedef signed char tS08; /*signed 8 bit definition */
// typedef int tS16; /*signed 16 bit definition*/


#define RXBufferSize 256 // have to be 2^n , 2,4,8,16,32,64,128,256
#ifndef SysClk
#define SysClk 8000000  // E-Clock 4MHz for UDM-EVB board, 8 or 24 for Techarts
#endif

// #define SPI 
#define PIXY_ARRAYSIZE              50
#define PIXY_START_WORD             0xaa55
#define PIXY_START_WORD_CC          0xaa56
#define PIXY_START_WORDX            0x55aa
#define PIXY_SERVO_SYNC             0xff
#define PIXY_CAM_BRIGHTNESS_SYNC    0xfe
#define PIXY_LED_SYNC               0xfd
#define PIXY_OUTBUF_SIZE            64

#define PIXY_SYNC_BYTE              0x5a
#define PIXY_SYNC_BYTE_DATA         0x5b

#define X_CENTER    160L
#define Y_CENTER    100L
#define RCS_MIN_POS     0L
#define RCS_MAX_POS     1000L
#define RCS_CENTER_POS	((RCS_MAX_POS-RCS_MIN_POS)/2)
//#define panKp = 200
//#define panKd = 200
//#define tiltKp = 150
//#define tiltKd = 150


// Function Prototypes
void panUpdate(int32_t error);
void tiltUpdate(int32_t error);
void ServoLoop(int32_t proportionalGain, int32_t derivativeGain);
int TrackBlock(int);
void FollowBlock(int);
void findWaitSpot(int);

// communication routines
// The use of "static" for these functions tells the compiler that they will
// only be called by other functions within this module which makes for 
// more efficient code
static uint16_t getWord(void);

// SPI sends as it receives so we need a getByte routine that 
// takes an output data argument
static uint8_t getByte(uint8_t out);
static int send(uint8_t *data, int len);

void init(void);    // Prototype for memory allocation
void initSPI(void);
 
int getStart(void);
uint16_t getBlocks(uint16_t maxBlocks);
int setServos(uint16_t s0, uint16_t s1);
int setBrightness(uint8_t brightness);
int setLED(uint8_t r, uint8_t g, uint8_t b);

// variables for a little circular queue used for getWord and Send in Pixy.c
// These variables are both declared and initialized.  The extern word by
// itself only does a declaration.  Without initialization here, one needs
// to initialize in the source C-file
extern uint8_t g_outBuf[PIXY_OUTBUF_SIZE];
extern uint8_t g_outLen;
extern uint8_t g_outWriteIndex;
extern uint8_t g_outReadIndex;


// variables for Block and Frame Parsing in Pixy.c
extern int g_skipStart;
extern BlockType g_blockType;
extern Block *g_blocks;



#endif