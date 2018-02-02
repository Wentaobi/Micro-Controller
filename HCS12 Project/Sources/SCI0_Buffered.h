/* SCI0.h
This is optimized for CodeWarrior HCS12 5.0
You must add the ISR routing to interrupt vector table to make it work
 
Author: Cheng-Lung Lee, 2012.Nov.28
*/

#ifndef _SCI0_Buffered_h
#define _SCI0_Buffered_h


void SCI0_Init(unsigned long Baud);
int putcharSCI0(char c);
int getcharSCI0(void);
int putcharSCI1(char c);
//int putchar(char c); 
void putArraySCI0(char size,char* ptr); 
void putsSCI0(char *ptr); // output string to SCI0
char getcharSCI0buffer(void);
unsigned char DataInSCI0buffer(void);
interrupt  void SCI0RX_ISR(void);  // SCI0 RX
void INIT_COMM_TRANSMIT(void);
void INIT_COMM_RECEIVE(void);

void SCI1_Init(unsigned long Baud);
int putcharSCI1(char c);
char getcharSCI1(void);
 
void putArraySCI1(char size,char* ptr); 
void putsSCI1(char *ptr); // output string to SCI0
char getcharSCI1buffer(void);
unsigned char DataInSCI1buffer(void);


#ifndef RXBufferSize
#define RXBufferSize 256 // have to be 2^n , 2,4,8,16,32,64,128,256
#endif

#ifndef SysClk
#define SysClk 8000000  // UDM-EVB 4M , TechArts:8M 
#endif

#ifndef SRXBuf
typedef struct
{
  unsigned char In ;    // 
  unsigned char Out ;    // 
   char buf[RXBufferSize];  // buffersize
} SRXBuf ;
#endif
extern unsigned char SCI1_bufin_index;
extern unsigned char CR_counter;

extern SRXBuf SCI1RXBuf;

#endif