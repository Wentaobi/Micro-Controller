//#include <hidef.h> 
#include "SCI0_Buffered.h"
#include "p2os.h"

#define SYNC0 0
#define SYNC1 1
#define SYNC2 2

#define PULSE 0
#define OPEN 1
#define CLOSE 2
#define ENABLE 4
#define SETA 5
#define SETV 6
#define SETO 7
#define MOVE 8
#define ROTATE 9
#define SETRV 10
#define VEL 11
#define HEAD 12
#define DHEAD 13
#define SAY 15
#define RVEL 21
#define DCHEAD 22
#define SETRA 23
#define SONAR 28
#define STOP 29
#define VEL2 32
#define GRIPPER 33
#define GRIPPERVAL 36
#define TTY2 42		// Added in AmigOS 1.2
#define GETAUX 43	// Added in AmigOS 1.2
#define BUMP_STALL 44
#define JOYDRIVE 47
#define SONARCYCLE 48
#define GYRO 58         // Added in AROS 1.8
#define ROTKP 82        // Added in P2OS1.M
#define ROTKV 83        // Added in P2OS1.M
#define ROTKI 84        // Added in P2OS1.M
#define TRANSKP 85      // Added in P2OS1.M
#define TRANSKV 86      // Added in P2OS1.M
#define TRANSKI 87      // Added in P2OS1.M
#define TTY3 66		// Added in AmigOS 1.3
#define GETAUX2 67	// Added in AmigOS 1.3
#define ARM_INFO 70
#define ARM_STATUS 71
#define ARM_INIT 72
#define ARM_CHECK 73
#define ARM_POWER 74
#define ARM_HOME 75
#define ARM_PARK 76
#define ARM_POS 77
#define ARM_SPEED 78
#define ARM_STOP 79
#define ARM_AUTOPARK 80
#define ARM_GRIPPARK 81
#define SOUND 90
#define PLAYLIST 91

/* Server Information Packet (SIP) types */
#define STATUSSTOPPED	0x32
#define STATUSMOVING	0x33
#define	ENCODER		0x90
#define SERAUX		0xB0
#define SERAUX2		0xB8	// Added in AmigOS 1.3
#define GYROPAC         0x98    // Added AROS 1.8
#define ARMPAC    160   // ARMpac
#define ARMINFOPAC  161   // ARMINFOpac
//#define PLAYLIST	0xD0

/* Argument types */
#define ARGINT		0x3B	// Positive int (LSB, MSB)
#define ARGNINT		0x1B	// Negative int (LSB, MSB)
#define ARGSTR		0x2B	// String (Note: 1st byte is length!!)

/* Pioneer 3DX message packages */
const char mySync1=0xfa;
const char mySync2=0xfb;
const char Sync0[6] = {0xfa, 0xfb, 0x03, 0x00, 0x00, 0x00 };
const char Sync1[6] = {0xfa, 0xfb, 0x03, 0x01, 0x00, 0x01 };
const char Sync2[6] = {0xfa, 0xfb, 0x03, 0x02, 0x00, 0x02 };
                   //index[0]   [1]   [2]   [3]   [4]   [5]   [6]   [7]   [8]
                        // 1     2     3     4     5     6     7     8     9
const char StopUltra[9] = {0xfa, 0xfb, 0x06, 0x1c, 0x3B, 0x00, 0x00, 0x1c, 0x3B };
const char SetAcc1[9]   = {0xfa, 0xfb, 0x06, 0x05, 0x3B, 0xe8, 0x03, 0xed, 0x3E };
const char SetAcc2[9]   = {0xfa, 0xfb, 0x06, 0x05, 0x1b, 0xe8, 0x03, 0xed, 0x1e };
const char DisMotor[9]  = {0xfa, 0xfb, 0x06, 0x04, 0x3B, 0x00, 0x00, 0x04, 0x3B };
const char ResetPos[7]  = {0xfa, 0xfb, 0x04, 0x07, 0x3B, 0x07, 0x3B             };
const char EnaMotor[9]  = {0xfa, 0xfb, 0x06, 0x04, 0x3B, 0x01, 0x00, 0x05, 0x3B };
//const char Drive1[9]  = {0xfa, 0xfb, 0x06, 0x04, 0x3B, 0x01, 0x00, 0x05, 0x3B };
//const char Drive2[9]  = {0xfa, 0xfb, 0x06, 0x04, 0x3B, 0x01, 0x00, 0x05, 0x3B };

void SendSonarEnable(void);
void ChangeSonarCycle(int);
void SendOPEN(void);

char myBuf[256];
char myLength=0;

/*
void main(void) {
  
  	EnableInterrupts;
    InitP3DX();
  for(;;) {

  DriveTest1();
    _FEED_COP(); // feeds the dog 
  } 
}
*/


// This function is modify from P3DX usermanual
int calcCheckSum(void)
{
  int i;
  unsigned char n;
  int c = 0;

  i = 3; // Start with 4th byte in myBuf
  // myBuf[] data buffer
  // myBuf[0] 0xFA
  // myBuf[1] 0xFB
  // myBuf[2] Packet Size
  // myBuf[...] Payload
  
  // get Packet Size without checksum (2 bytes)
  n = myBuf[2] - 2;
  while (n > 1) {
    c += ((unsigned char)myBuf[i]<<8) | (unsigned char)myBuf[i+1];
    c = c & 0xffff;
    n -= 2;
    i += 2;
  }
  if (n > 0) 
    c = c ^ (int)((unsigned char) myBuf[i]);
  return c;
}

void finalizePacket(void)
{
  int chkSum;

  myBuf[0]=mySync1;
  myBuf[1]=mySync2;
  myBuf[2]=myLength - 3;

  chkSum = calcCheckSum();

  myBuf[myLength-2]=((chkSum >> 8) & 0xff );
  myBuf[myLength-1]=(chkSum & 0xff );
  
}

void delayms(unsigned short ms)
{
	// This only works for 4MHz ECLOCK
  volatile unsigned short i; /* volatile so compiler does not optimize */
  while (ms > 0)
  {
  i = 331;
  /* ------------------------ */
  while (i > 0)  i--;
  ms--;
  _asm("nop");
  _asm("nop");
  _asm("nop");
  _asm("nop");
  //_asm("nop");

  }
}

void safedelayms( unsigned short ms) {
	unsigned short i,j;
	i=ms/100;
	j=ms%100;
	
	while(i>0){
		i--;  
		delayms(100);
		putArraySCI0(6,&Sync0[0]);
	}
	
	if (j>0){
		delayms(j);
		putArraySCI0(6,&Sync0[0]);
	}
}


void SendVEL2(int RSpeed,int LSpeed)
{
  
  
 // test VEL2 package 
  myLength=9;
  myBuf[3]=VEL2; // VEL2 command
  myBuf[4]=ARGINT; // argument type
  myBuf[5]=RSpeed/20; // argument Right wheel speed 20mm/s/unit
  myBuf[6]=LSpeed/20; // argument Left  wheel speed 20mm/s/unit
//  myBuf[7]=0x00; Checksum
//  myBuf[8]=0x00; Checksum
  finalizePacket();
  putArraySCI0(myLength,&myBuf[0]);
} 
void SendVEL(int Speed)
{
  
  
 // test VEL package 
  myLength=9;
  myBuf[3]=VEL; // VEL command
  myBuf[4]=ARGINT; // argument type
  myBuf[5]=Speed & 0xFF; // argument       speed mm/s
  myBuf[6]=(Speed>>8) & 0xff; // argument  speed mm/s
//  myBuf[7]=0x00; Checksum
//  myBuf[8]=0x00; Checksum
  finalizePacket();
  putArraySCI0(myLength,&myBuf[0]);
}

void SendROTATE(int TurnRate)
{
  
  
 // test rotate package 
  myLength=9;
  myBuf[3]=ROTATE; // ROTATE command
  myBuf[4]=ARGINT; // argument type
// int low byte first
  myBuf[5]=TurnRate & 0xFF; // argument Left  wheel speed 20mm/s/unit
  myBuf[6]=(TurnRate>>8) & 0xff; // argument Right wheel speed 20mm/s/unit
//  myBuf[7]=0x00; Checksum
//  myBuf[8]=0x00; Checksum
  finalizePacket();
  putArraySCI0(myLength,&myBuf[0]);
}
void SendOPEN(void)
{ 
 // test rotate package 
  myLength=9;
  myBuf[3]=OPEN; // ROTATE command
  myBuf[4]=0xFF;
  finalizePacket();
  putArraySCI0(myLength,&myBuf[0]);
}
void SendMOVE(int Dist)
{
  
  
 // generate move package 
  myLength=9;
  myBuf[3]=MOVE; // MOVE command
  myBuf[4]=ARGINT; // argument type
// int low byte first
  myBuf[5]=Dist & 0xFF; // argument Left  wheel speed 20mm/s/unit
  myBuf[6]=(Dist>>8) & 0xff; // argument Right wheel speed 20mm/s/unit
//  myBuf[7]=0x00; Checksum
//  myBuf[8]=0x00; Checksum
  finalizePacket();
  putArraySCI0(myLength,&myBuf[0]);
}

void SendDHEAD(int Data)
{
  
  
 // generate move package 
  myLength=9;
  myBuf[3]=DHEAD; // DHEAD command
  myBuf[4]=ARGINT; // argument type
// int low byte first
  myBuf[5]=Data & 0xFF; // argument Left  wheel speed 20mm/s/unit
  myBuf[6]=(Data>>8) & 0xff; // argument Right wheel speed 20mm/s/unit
//  myBuf[7]=0x00; Checksum
//  myBuf[8]=0x00; Checksum
  finalizePacket();
  putArraySCI0(myLength,&myBuf[0]);
}

void SendSonarEnable(void)
{
  
  
 // generate package 
  myLength=9;
  myBuf[3]=SONAR;    // Enable SONAR Command
  myBuf[4]=ARGINT; // Positive int (LSB, MSB)
  myBuf[5]=0xFF;      // Turn SONARs On
  finalizePacket();
  putArraySCI0(myLength,&myBuf[0]);
}
void ChangeSonarCycle(int time)
{
 // generate package 
  myLength=9;
  myBuf[3]=SONARCYCLE;    // Enable SONAR Command
  myBuf[4]=ARGINT; // Positive int (LSB, MSB)
  myBuf[5]=time;      // Turn SONARs On
  finalizePacket();
  putArraySCI0(myLength,&myBuf[0]);
}
void SendINT(char CMD, int Data)
{  
 // generate package 
  myLength=9;
  myBuf[3]=CMD;    // command
  myBuf[4]=ARGINT; // Positive int (LSB, MSB)
  myBuf[5]=Data & 0xFF;      // LSB
  myBuf[6]=(Data>>8) & 0xff; // MSB
  finalizePacket();
  putArraySCI0(myLength,&myBuf[0]);
}
void SendNINT(char CMD, int Data)
{
 // generate package 
  myLength=9;
  myBuf[3]=CMD; 
  myBuf[4]=ARGNINT; // Negative int (LSB, MSB)
  myBuf[5]=Data & 0xFF;      // LSB
  myBuf[6]=(Data>>8) & 0xff; // MSB
  finalizePacket();
  putArraySCI0(myLength,&myBuf[0]);
}

void InitP3DX(void){

    delayms(2000);   // Startup delay , waite for P3DX to boot.
    // Sync to P3DX
    putArraySCI0(6,&Sync0[0]);
    delayms(100);
    putArraySCI0(6,&Sync1[0]);
    delayms(100);
    putArraySCI0(6,&Sync2[0]);
    delayms(100);
    SendOPEN();
    putArraySCI0(6,&Sync0[0]);
    delayms(100);
    putArraySCI0(6,&Sync1[0]);
    delayms(100);
    // Setup Acceleration parameters for P3Dx motor
    putArraySCI0(9,&SetAcc1[0]);
    delayms(100);
    putArraySCI0(9,&SetAcc2[0]);
    delayms(100);
    // Enable Motor
    putArraySCI0(9,&EnaMotor[0]);
    delayms(100);

}


void DriveTest1(void) {
	int i;
	delayms(100);
	SendINT(SETV,100); // set max translate speed 200mm/s
	delayms(100);
	SendINT(SETRV,30); // set max turn rate 100 degree/s
	delayms(500);
	
	// drive square , use safedelayms() to keep feeding watchdog
	
	SendMOVE(500);
	safedelayms( 5500);
	SendDHEAD(90);
	safedelayms( 3500);
	
	SendMOVE(500);
	safedelayms( 5500);
	SendDHEAD(90);
	safedelayms( 3500);
	
	SendMOVE(500);
	safedelayms( 5500);
	SendDHEAD(90);
	safedelayms( 3500);
	
	SendMOVE(500);
	safedelayms( 5500);
	SendDHEAD(90);
	safedelayms( 3500);

}

void DriveTest2(void) {
  int i=0;
  while(i<100) {
      i++;
      SendROTATE(i);
      delayms(100);
  }

  while(i>-100) {
      i--;
      SendROTATE(i);
      delayms(100);
  }
  while(i<0) {
      i++;
      SendROTATE(i);
      delayms(100);
  }

}

void DriveTest3(void) {
  int i=0;
  while(i<100) {
      i++;
      SendVEL(i);
      delayms(100);
  }

  while(i>-100) {
      i--;
      SendVEL(i);
      delayms(100);
  }
    while(i<0) {
      i++;
      SendVEL(i);
      delayms(100);
  }

}
