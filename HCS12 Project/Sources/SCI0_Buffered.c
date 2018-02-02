/*############################### Project Header ################################
; Course: ELEE5870 Laboratory
; Project Name: Final Project
; Tilte: This is optimized for CodeWarrior HCS12 5.0
; You must add the ISR routing to interrupt vector table to make it work		
; Organization:	University of Detroit Mercy
; Author:Cheng-Lung Lee,
; Co-author:  	

; Abstract: In this module, we try to solve the following problems:
; To initialize and setup the SCI0 and SCI1 communication between the p2os and MCU.
; To establish transmit and receive subroutine to transmit and receive data via SCI 

 
;******************************** End of Header ********************************


;############################## Copyright Notice ###############################
; This is an un-published work created in 2013 at the University of Detroit 
; Mercy (UDM). Reproduction of this work electronically or by any other means,
; without the consent of UDM faculty, is a violation of copyright law and
; the student code of conduct. Persons providing, receiving, or possessing this
; information without appropriate permission are conducting an act of academic
; dishonesty with consequences up to and including expulsion from this 
; University, or any university, school, or educational community with similar
; standards for academic integrity.

; 

;****************************** End of Copyright *******************************


;############################### Version History ###############################
; Start Date:	YYYY/MM/DD	(Date Format: YYYY/MM/DD)
; Latest Date:	2013/11/30
; Version:  	0.1                                                                
;
; Date:		By/Reviewed:	Description of changes: Reproduce the SCI1 communication 
; 2013/11/30 Dingwang Wang 	Initial Release

;******************************* End of History *********************************/



/* SCI0.c
This is optimized for CodeWarrior HCS12 5.0
You must add the ISR routing to interrupt vector table to make it work
    */

#include <hidef.h>      /* common defines and macros */
#include "derivative.h"      /* derivative-specific definitions */
#include "SCI0_Buffered.h"
#include "PixyHS12.h"


SRXBuf SCI0RXBuf;
SRXBuf SCI1RXBuf;
SRXBuf SPI0RXBuf;

unsigned char SCI1_buf[RXBufferSize];  // buffersize
unsigned char SCI1_bufin_index;
unsigned char Prompt_Is_Ready;

extern int Obstacle;

void SPI0_Init(void);
int putcharSPI0(char);
int getcharSPI0(void);


void SPI0_Init(void)
{
   // initialize buffer
   SPI0RXBuf.In=0;
   SPI0RXBuf.Out=0;
        
   /* Variable Declarations */
   /* Begin Function InitSPI() */
   /* using the system clock rate */
   SPI0CR1 = SPI0CR1_SPE + SPI0CR1_MSTR + SPI0CR1_CPHA; /* enable both the transmitter & receiver, only RX interrupt */

} /* end InitSPI */

void SCI0_Init(unsigned long Baud)
{
   // initialize buffer
   SCI0RXBuf.In=0;
   SCI0RXBuf.Out=0;
   
   /* Variable Declarations */
   /* Begin Function InitSCI() */
   SCI0BD = (SysClk / 16) / Baud; /* calculate the SCI Baud register value */
   /* using the system clock rate */
   SCI0CR2 = SCI0CR2_RIE_MASK + SCI0CR2_TE_MASK + SCI0CR2_RE_MASK ; /* enable both the transmitter & receiver, only RX interrupt */
   //SCI0CR2 = SCI0CR2_TE_MASK + SCI0CR2_RE_MASK ; /* enable both the transmitter & receiver, no interrupt */
} /* end InitSCI */


void putcharSCI0(char c)
{
	while (!(SCI0SR1&SCI0SR1_TDRE_MASK)) /* check TXbuffer is ready or not,TDRE */
	//OSTimeDly(1); /* wait here */
	SCI0DRL=c; /* put the char in the TX */
//	return(c); /* return the character we sent */
} /* end putchar */

int getcharSCI0(void)
{
   while (!(SCI0SR1&SCI0SR1_RDRF_MASK)) /* chec RX flag, has data or not...,RDRF */
   //OSTimeDly(1); /* no data wait here */
   return(SCI0DRL);
} /* end getchar */

int putcharSPI0(char c)
{
	while (!(SPI1SR&SPI1SR_SPTEF_MASK)) /* check TXbuffer is ready or not,TDRE */
	; //OSTimeDly(1); /* wait here */
	SPI1DR=c; /* put the char in the TX */
	return(c); /* return the character we sent */
} /* end putchar */

int getcharSPI0(void)
{
   while (!(SPI1SR&SPI1SR_SPIF_MASK)) /* chec RX flag, has data or not...,RDRF */
   //OSTimeDly(1); /* no data wait here */
   return(SPI1DR);
} /* end getchar */

void putArraySCI0(char size,char* ptr) // send binary array to SPI0
{
	while(size--) 	putcharSCI0(*ptr++);
}

void putsSCI0(char *ptr) // output string to SCI0
{
	while(*ptr){
	putcharSCI0(*ptr);
	ptr++;
	}
}
char getcharSCI0buffer(void)
{
   return SCI0RXBuf.buf[SCI0RXBuf.Out++]; // just read data ( no checking in/out index here)
   SCI0RXBuf.Out&=(RXBufferSize-1); // limite in max index size
}

unsigned char DataInSCI0buffer(void)
{
   return ((SCI0RXBuf.In-SCI0RXBuf.Out)&(RXBufferSize-1)); // get difference in index
}

/*
 Note: must add the ISR to interrupt vector table
 IVT at Project Settings\Linker Files\Project.prm
 Address  Index Notes
 0xFFD4   21    SCI1 ISR
 0xFFD6   20    SCI0 ISR
 
 in end of prm add
	VECTOR 20 SCI0RX_ISR
 in prm ENTRIES add
    SCI0RX_ISR
 
 Note: the ISR code must in none-banked segment
 Here is the example
 
 #pragma CODE_SEG __NEAR_SEG NON_BANKED
 interrupt void irqISR (void)
 {
 int i=0;// statements to service the interrupt
 }
 #pragma CODE_SEG DEFAULT                                                

*/

#pragma CODE_SEG __NEAR_SEG NON_BANKED
interrupt 20 void SCI0RX_ISR(void)  // Interrupt Service Routine SCI0
{
	if (SCI0SR1&SCI0SR1_RDRF_MASK) /* chec RX flag, has data or not...,RDRF */
	{
	 //	char C=SCI0DRL;
	 SCI0RXBuf.buf[SCI0RXBuf.In++]=SCI0DRL; // Put data into SCI buffer
	 SCI0RXBuf.In &= (RXBufferSize-1);// Make sure index value are in the range ( mask to force overflow )
  if(SCI0RXBuf.In==SCI0RXBuf.Out) // checking for buffer overflow , too much data in buffer
   SCI0RXBuf.Out= (SCI0RXBuf.Out+1 )& (RXBufferSize-1); // output index have to +1 & limite in range
	 SCI0SR1|=SCI0SR1_RDRF_MASK; // clear RX flag
  }
return;
}
#pragma CODE_SEG DEFAULT


void SCI1_Init(unsigned long Baud)
{
   // initialize buffer
  
   
   /* Variable Declarations */
   /* Begin Function InitSCI() */
   SCI1BD = (SysClk / 16) / Baud; /* calculate the SCI Baud register value */
   /* using the system clock rate */
  SCI1CR2 = SCI1CR2_RIE_MASK + SCI1CR2_TE_MASK + SCI1CR2_RE_MASK ; /* enable both the transmitter & receiver, only RX interrupt */
   //SCI0CR2 = SCI0CR2_TE_MASK + SCI0CR2_RE_MASK ; /* enable both the transmitter & receiver, no interrupt */
} /* end InitSCI */


int putcharSCI1(char c)
{
	while (!(SCI1SR1&SCI0SR1_TDRE_MASK)) /* check TXbuffer is ready or not,TDRE */
	; //OSTimeDly(1); /* wait here */
	SCI1DRL=c; /* put the char in the TX */
	return(c); /* return the character we sent */
} /* end putchar */

int getcharSCI1(void)
{
   while (!(SCI1SR1&SCI1SR1_RDRF_MASK)) /* chec RX flag, has data or not...,RDRF */
   //OSTimeDly(1); /* no data wait here */
   return(SCI1DRL);
} /* end getchar */

void putArraySCI1(char size,char* ptr) // send binary array to SCI1
{
	while(size--) 	putcharSCI1(*ptr++);
}

void putsSCI1(char *ptr) // output string to SCI1
{
	while(*ptr){
	putcharSCI1(*ptr);
	ptr++;
	}
}






/**********************************************************
*	sci_isr
*
*	interrupt handler for SCI1 module interrupts
**********************************************************/
#pragma CODE_SEG __NEAR_SEG NON_BANKED
interrupt 21 void SCI1RX_ISR(void)  // Interrupt Service Routine SCIa
  {
    
    	if (SCI1SR1&SCI1SR1_RDRF_MASK) /* chec RX flag, has data or not...,RDRF */
	{
	 //	char C=SCI0DRL;
	 SCI1RXBuf.buf[SCI1_bufin_index++]=SCI1DRL; // Put data into SCI buffer
	 SCI1_bufin_index &= (RXBufferSize-1);// Make sure index value are in the range ( mask to force overflow )
   if(SCI1DRL==0x3A) 
   {
    
   Prompt_Is_Ready=1;                     // checking if it is prompt symbol colon":",if yes then set the flag
   }
   
   	 
   	 
   	 
	 SCI1SR1|=SCI1SR1_RDRF_MASK; // clear RX flag
	}
	


//	EnableInterrupts;
}
#pragma CODE_SEG DEFAULT
