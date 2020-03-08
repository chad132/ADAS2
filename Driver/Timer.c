#include "Timer.h"
#include "USART.h"
#include "delay.h"
#include "CAN.h"
#include "LED.h"

uint8_t Flag_Voice_Start,Flag_Voice_Start1,Flag_Voice_End = 0;
uint8_t SignalState,SignalStateNew=0;





/**-------------------------------------------------------
  * @函数名 TIM2_Init()
  * @功能   100ms定时
  * @参数   无
  * @返回值 无
***------------------------------------------------------*/
void TIM2_Init(void)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    /* TIM2 clock enable */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
		    
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;//1 0
	  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);


    /* 自动重装载寄存器周期的值(计数值) */ 
		TIM_TimeBaseStructure.TIM_Period = (1000 - 1);//10:1ms 20:2ms  10000:1s

	/* 累计 TIM_Period个频率后产生一个更新或者中断 */
    /* 这个就是预分频系数*/
		TIM_TimeBaseStructure.TIM_Prescaler = (7200 - 1);//72M:7200   10k

    /* 采样分频 */
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;

    /* 向上计数模式 */
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

    /* 初始化定时器2 */
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

    /* 清除溢出中断标志 */
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);

    /* 溢出中断使能 */
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

    /* 计数器使能 */
    TIM_Cmd(TIM2, ENABLE); 
	
}


/**-------------------------------------------------------
  * @函数名 TIM4_Init()
  * @功能   Timer4定时50ms扫描AUTO旋钮、雾灯开关、仪表灯亮度编码盘、大灯角度编码盘键值
  * @参数   无
  * @返回值 无
***------------------------------------------------------*/
void TIM4_Init(void)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    /* TIM4 clock enable */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
		    
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;//2 0
	  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);


    /* 自动重装载寄存器周期的值(计数值) */ 
		TIM_TimeBaseStructure.TIM_Period = (100 - 1);//10:1ms 20:2ms  10000:1s

	  /* 累计 TIM_Period个频率后产生一个更新或者中断 */
    /* 这个就是预分频系数*/
		TIM_TimeBaseStructure.TIM_Prescaler = (7200 - 1);//72M:7200

    /* 采样分频 */
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;

    /* 向上计数模式 */
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

    /* 初始化定时器2 */
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

    /* 清除溢出中断标志 */
    TIM_ClearITPendingBit(TIM4, TIM_IT_Update);

    /* 溢出中断使能 */
    TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);

    /* 计数器使能 */
    TIM_Cmd(TIM4, ENABLE); 
	
}



void TIM2_IRQHandler(void)//基础定时：100ms
{
	static uint8_t i,j,can_num;
  if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
    {			
      TIM_ClearITPendingBit(TIM2, TIM_IT_Update);

			CAN_TX();//CAN发送

			//		if((Flag_FCW == 1) && (Flag_LDW == 1))//同时相应
//		{
//		  Flag_Alarm = 3;
//		}
		if(Flag_FCWLamp == 1)//FCW相应
		{
		  Flag_Alarm = 1;
		}
		else if(Flag_LDWLamp == 1)//LDW相应
		{
		  Flag_Alarm = 2;
		}
    else               //无报警
		{
		  Flag_Alarm = 0;
		}	



		
		Data_CAR_Process();//车辆信息串口发送
		
		}
}


void TIM4_IRQHandler(void)//10ms用于按键扫描
{
	
		uint8_t IO_State = 0;
	  static uint8_t Num,Num_LeftTurn,Num_RightTurn ;
	  static uint8_t Num_Red,Num_4,Flag_Red;
  if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)
   {
		 TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
		 
	    Data_Process();//通过串口检测到模块的数据后处理
		  if((Flag_Alarm == 1) || (Flag_Alarm == 2))//FCW/LDW、红灯、蓝灯标志位
			{
				if(Flag_Red == 0)//4次闪烁完成后才能开始初始化
				{
					Flag_Red = 1;
					Num_4 = 0;
					Num_Red = 0;				
				}

			}
			else
			{
			  LED_GREEN_ON();//绿灯
			}

		   if(Flag_Red == 1)//闪烁4次标志位
			 {
				 Num_Red++;
				 if(Num_Red == 40)
				 {
					 Num_Red = 0;
					 Num_4++;
					 if(Num_4 == 4)//红灯闪4次
					 {
						 Num_4 = 0;
						 Flag_Red = 0;//闪烁完成
						 Flag_FCWLamp = 0; //标志位清0		
		         Flag_LDWLamp = 0;	

					 }
				 }
			 
				if(Num_Red <= 20)
				{
					
					LED_OFF();
				}
				else 
				{
					if(Flag_Alarm == 1)//FCW报警，红灯亮
					{
				    LED_RED_ON();					
					}
					else if(Flag_Alarm == 2)//LDW报警，蓝灯亮
          {
					  LED_BLUE_ON();
					}
				}				 
			 }
			


	
					
	 }

}