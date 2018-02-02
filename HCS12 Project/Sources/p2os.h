#ifndef _p2ox_h
#define _p2os_h
//#include <hidef.h>      /* common defines and macros */
         
/* p2os.h Command numbers */

int  calcCheckSum(void);
void finalizePacket(void);
void delayms(unsigned short num);
void safedelayms( unsigned short ms);
void SendVEL2(int RSpeed,int LSpeed);
void SendVEL(int Speed) ;
void SendROTATE(int TurnRate);
void SendDHEAD(int Data) ;
void SendMOVE(int Dist);
void SendINT(char CMD, int Data);
void SendNINT(char CMD, int Data);
void InitP3DX(void);
void DriveTest1(void);
void DriveTest2(void);
void DriveTest3(void);



#endif