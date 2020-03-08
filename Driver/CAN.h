#include "stm32f10x.h"



#define ENGINESPEED_ID    0x18FFA017  //车速,油门
#define LEMP_ID           0x18FFA217 //夜灯、左右转向灯
#define START_ID          0x0CFA00D0 //点火START

#define SYSTEMDETECT          0x01 //系统自检
#define SYSTENORMAL           0x02 //系统正常
#define SYSTEFAULT            0x03 //设备故障

//#define START_ON 0x03
//#define LEFTTURN_ON 0x01
//#define RIGHTTURN_ON 0x01
//#define NIGHTLAMP_ON 0x01

#define THRESHOLD_SPEED 20

extern uint16_t Date_Speed ;
extern uint8_t Date_Engine,Date_NightLight,Date_LeftTurn,Date_RightTurn ;
extern uint8_t Flag_CAN_Speed,Flag_CAN_Engine,Flag_CAN_Lamp ;
extern uint8_t Flag_FirstSartL,Flag_FirstSartR;
extern uint8_t Flag_Alarm;
extern uint8_t Flag_FCWLamp,Flag_LDWLamp;

void CANInit(void);
uint8_t CAN_TX(void);
//int Serial_Datas(void);
void Data_Process(void);