#include "stm32f10x.h"
#include <stdio.h>
#include "USART.h"
#include "delay.h"
#include "CAN.h"
#include "LED.h"
#include "Timer.h"
uint8_t Flag_dw3 = 0;

RCC_ClocksTypeDef RCC_Clocks;
int main(void)
{
	
	 Delay_Init(72);//滴答时钟初始化
	 NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//配置中断为分组2
	 LED_Init();
	 CANInit();
	 TIM2_Init();
	 TIM4_Init();
	 USART1_Init();
	 RCC_GetClocksFreq(&RCC_Clocks);//debug模式下读取各个模块的时钟
	 Flag_dw3 =5;
	 
   while(1)
   {
//   	LED_RED_ON();
//		Delay_Ms(500);
//		LED_GREEN_ON();
//		Delay_Ms(500); 
//    LED_BLUE_ON();		 
//		Delay_Ms(500); 
		 

		 
		 
//		 if(Flag_Alarm == 0)//无报警，绿灯亮
//		 {
//		   LED_GREEN_ON();
//		 }
//		 else if(Flag_Alarm == 1)//FCW报警，红灯亮
//		 {
//		   LED_RED_ON();
//		 }
//		 else if(Flag_Alarm == 2)//LDW报警，蓝灯亮
//		 {
//		   LED_BLUE_ON();
//		 }
//		 else if(Flag_Alarm == 3)//全报警，
//		 {
//		   LED_REDBLUE_ON();
//		 }
   }
	 
	 
}