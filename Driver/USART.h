#include "stm32f10x.h"


extern uint8_t Voice_Num;
extern uint8_t Param[12];

void USART1_Init(void);
void Txd(unsigned char byte);
void USART1_SendArry(unsigned char *pdata, unsigned char len);
unsigned short MakeCRC(unsigned char * ptr,unsigned char len);
void Data_Calibration_Process(void);


void Data_CAR_Process(void);
void Data_CAR_Send(void);
//void JQ8400_Play(uint8_t Num);
//void JQ8400_Volume(uint8_t Num);
//void JQ8400_Play_Stop(void);
//void USART2_TxHigh(void);
//void USART2_TxLow(void);

#define FCW_CARNO 0x00
#define FCW_CARSAFETY 0x0E
#define FCW_CARALARMSAFETY 0x0F
#define FCW_CARALARMFIRST 0x06
#define FCW_CARALARMSECOND 0x05
#define FPW 0x0a
#define FVSM 0x0b
#define FCW_CARALARMREMOVE 0x10

#define LDW_ALARMNO  0x00
#define LDW_ALARMFULLLINELEFT  0x02
#define LDW_ALARMDASHEDLEFT  0x01
#define LDW_ALARMFULLLINERIGHT  0x04
#define LDW_ALARMDASHEDRIGHT  0x03


