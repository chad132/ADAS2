#include "Timer.h"
#include "USART.h"
#include "delay.h"
#include "CAN.h"
#include "LED.h"

uint8_t Flag_Voice_Start,Flag_Voice_Start1,Flag_Voice_End = 0;
uint8_t SignalState,SignalStateNew=0;





/**-------------------------------------------------------
  * @������ TIM2_Init()
  * @����   100ms��ʱ
  * @����   ��
  * @����ֵ ��
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


    /* �Զ���װ�ؼĴ������ڵ�ֵ(����ֵ) */ 
		TIM_TimeBaseStructure.TIM_Period = (1000 - 1);//10:1ms 20:2ms  10000:1s

	/* �ۼ� TIM_Period��Ƶ�ʺ����һ�����»����ж� */
    /* �������Ԥ��Ƶϵ��*/
		TIM_TimeBaseStructure.TIM_Prescaler = (7200 - 1);//72M:7200   10k

    /* ������Ƶ */
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;

    /* ���ϼ���ģʽ */
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

    /* ��ʼ����ʱ��2 */
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

    /* �������жϱ�־ */
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);

    /* ����ж�ʹ�� */
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

    /* ������ʹ�� */
    TIM_Cmd(TIM2, ENABLE); 
	
}


/**-------------------------------------------------------
  * @������ TIM4_Init()
  * @����   Timer4��ʱ50msɨ��AUTO��ť����ƿ��ء��Ǳ�����ȱ����̡���ƽǶȱ����̼�ֵ
  * @����   ��
  * @����ֵ ��
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


    /* �Զ���װ�ؼĴ������ڵ�ֵ(����ֵ) */ 
		TIM_TimeBaseStructure.TIM_Period = (100 - 1);//10:1ms 20:2ms  10000:1s

	  /* �ۼ� TIM_Period��Ƶ�ʺ����һ�����»����ж� */
    /* �������Ԥ��Ƶϵ��*/
		TIM_TimeBaseStructure.TIM_Prescaler = (7200 - 1);//72M:7200

    /* ������Ƶ */
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;

    /* ���ϼ���ģʽ */
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

    /* ��ʼ����ʱ��2 */
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

    /* �������жϱ�־ */
    TIM_ClearITPendingBit(TIM4, TIM_IT_Update);

    /* ����ж�ʹ�� */
    TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);

    /* ������ʹ�� */
    TIM_Cmd(TIM4, ENABLE); 
	
}



void TIM2_IRQHandler(void)//������ʱ��100ms
{
	static uint8_t i,j,can_num;
  if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
    {			
      TIM_ClearITPendingBit(TIM2, TIM_IT_Update);

			CAN_TX();//CAN����

			//		if((Flag_FCW == 1) && (Flag_LDW == 1))//ͬʱ��Ӧ
//		{
//		  Flag_Alarm = 3;
//		}
		if(Flag_FCWLamp == 1)//FCW��Ӧ
		{
		  Flag_Alarm = 1;
		}
		else if(Flag_LDWLamp == 1)//LDW��Ӧ
		{
		  Flag_Alarm = 2;
		}
    else               //�ޱ���
		{
		  Flag_Alarm = 0;
		}	



		
		Data_CAR_Process();//������Ϣ���ڷ���
		
		}
}


void TIM4_IRQHandler(void)//10ms���ڰ���ɨ��
{
	
		uint8_t IO_State = 0;
	  static uint8_t Num,Num_LeftTurn,Num_RightTurn ;
	  static uint8_t Num_Red,Num_4,Flag_Red;
  if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)
   {
		 TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
		 
	    Data_Process();//ͨ�����ڼ�⵽ģ������ݺ���
		  if((Flag_Alarm == 1) || (Flag_Alarm == 2))//FCW/LDW����ơ����Ʊ�־λ
			{
				if(Flag_Red == 0)//4����˸��ɺ���ܿ�ʼ��ʼ��
				{
					Flag_Red = 1;
					Num_4 = 0;
					Num_Red = 0;				
				}

			}
			else
			{
			  LED_GREEN_ON();//�̵�
			}

		   if(Flag_Red == 1)//��˸4�α�־λ
			 {
				 Num_Red++;
				 if(Num_Red == 40)
				 {
					 Num_Red = 0;
					 Num_4++;
					 if(Num_4 == 4)//�����4��
					 {
						 Num_4 = 0;
						 Flag_Red = 0;//��˸���
						 Flag_FCWLamp = 0; //��־λ��0		
		         Flag_LDWLamp = 0;	

					 }
				 }
			 
				if(Num_Red <= 20)
				{
					
					LED_OFF();
				}
				else 
				{
					if(Flag_Alarm == 1)//FCW�����������
					{
				    LED_RED_ON();					
					}
					else if(Flag_Alarm == 2)//LDW������������
          {
					  LED_BLUE_ON();
					}
				}				 
			 }
			


	
					
	 }

}