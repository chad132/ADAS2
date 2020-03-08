#include "CAN.h"
#include "USART.h"

CanTxMsg TxMessage;
CanRxMsg RxMessage;

uint8_t CAN_Data7 = 0;
uint8_t Flag_Alarm = 0;//������־λ
uint8_t Flag_CAN_EngineSpeed= 0;//��־λ����
uint16_t Date_Speed = 0;
uint8_t Date_LeftTurn,Date_RightTurn = 0;
uint8_t Flag_CAN_EngineSpeed,Flag_CAN_LEMP = 0;



unsigned char Flag_FCW;       //FCW��־
unsigned char Time_HMW;       //��ǰ��ʱ��
unsigned int  Distance_FCW;   //��ǰ������
unsigned char Flag_LDW;       //LDW��־
unsigned int  Distance_LDWL;  //���־���
unsigned int  Distance_LDWR;  //���־���

uint8_t Flag_FCWLamp,Flag_LDWLamp;

void CANInit(void)
{
	
	
   GPIO_InitTypeDef  GPIO_InitStructure;
	 CAN_InitTypeDef  CAN_InitStructure;	 
	 CAN_FilterInitTypeDef  CAN_FilterInitStructure;
	 NVIC_InitTypeDef  NVIC_InitStructure;
	
	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA  | RCC_APB2Periph_AFIO, ENABLE);//ʹ��PORTAʱ��	                   											 
	 RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);//ʹ��CAN1ʱ��
	
	 //***CAN-TX  IO Init***//
	 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;		//�����������
 	 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_Init(GPIOA, &GPIO_InitStructure);
	
		
	 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;		//CAN_SҪ����
 	 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	
	 //***CAN-RX  IO Init***//
	 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;			//��������
	 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
 	 GPIO_Init(GPIOA, &GPIO_InitStructure);
	 
	 CAN_DeInit(CAN1);		//��ʼ����
	 CAN_StructInit(&CAN_InitStructure);
	 
	 CAN_InitStructure.CAN_TTCM=DISABLE;           // ʱ�䴥��ͨ�Ž�ֹ
 	 CAN_InitStructure.CAN_ABOM=ENABLE;            // һ��Ӳ����⵽128��11λ����������λ�����Զ��˳�����״̬
 	 CAN_InitStructure.CAN_AWUM=DISABLE;           // �Զ�����ģʽ������sleep
 	 CAN_InitStructure.CAN_NART=DISABLE;           // �Զ����´��ͱ��ģ�ֱ�����ͳɹ�
 	 CAN_InitStructure.CAN_RFLM=DISABLE;           // FIFOû���������±��ĸ��Ǿɱ���
 	 CAN_InitStructure.CAN_TXFP=DISABLE;           // ���ͱ������ȼ�ȷ������־��ȷ��
 	 CAN_InitStructure.CAN_Mode=CAN_Mode_Normal;   // ģʽ: CAN_Mode_Normal /CAN_Mode_LoopBack/CAN_Mode_Silent_LoopBack
	 CAN_InitStructure.CAN_SJW=CAN_SJW_1tq;        // SWJ:(0-4)
	 CAN_InitStructure.CAN_BS1=CAN_BS1_4tq;				 // BS1:(0-16)
	 CAN_InitStructure.CAN_BS2=CAN_BS2_3tq;				 // BS2:(0-8)
	 CAN_InitStructure.CAN_Prescaler=18;           // ��Ƶ��:([9:0]) CAN������={36MHZ/(1+4+3)}/18=250KB
	 CAN_Init(CAN1, &CAN_InitStructure);           // ��ʼ��CAN1
	 
	 /************��Ҫ�õ�����������**************/
	 
	 CAN_FilterInitStructure.CAN_FilterNumber=0;			//������0,���ڹ�������̤��ͳ��ٱ���
	 CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdList; //�б�ģʽ
	 CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;
	 CAN_FilterInitStructure.CAN_FilterIdHigh=((ENGINESPEED_ID<<3) >>16) &0xffff;				//������չ֡ID��λ
	 CAN_FilterInitStructure.CAN_FilterIdLow=(uint16_t)(ENGINESPEED_ID<<3) | CAN_ID_EXT;//��չ֡ID��λ
	 CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0xffff;
	 CAN_FilterInitStructure.CAN_FilterMaskIdLow=0xffff;				//
	 CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_FIFO0; //������0
	 CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;			 //ENABLE;
	 CAN_FilterInit(&CAN_FilterInitStructure);
	 
	 CAN_FilterInitStructure.CAN_FilterNumber=1;			//������1������Start�źű���
	 CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdList; //�б�ģʽ
	 CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;
	 CAN_FilterInitStructure.CAN_FilterIdHigh=((START_ID<<3) >>16) &0xffff;				//������չ֡ID��λ
	 CAN_FilterInitStructure.CAN_FilterIdLow=(uint16_t)(START_ID<<3) | CAN_ID_EXT;//��չ֡ID��λ
	 CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0xffff;
	 CAN_FilterInitStructure.CAN_FilterMaskIdLow=0xffff;				//ID������ȫƥ��
	 CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_FIFO0; //������0
	 CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;			 //ENABLE;
	 CAN_FilterInit(&CAN_FilterInitStructure);
	 
	 CAN_FilterInitStructure.CAN_FilterNumber=2;			//������2�����ڹ�������ת����źű���
	 CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdList; //�б�ģʽ
	 CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;
	 CAN_FilterInitStructure.CAN_FilterIdHigh=((LEMP_ID<<3) >>16) &0xffff;				//������չ֡ID��λ
	 CAN_FilterInitStructure.CAN_FilterIdLow=(uint16_t)(LEMP_ID<<3) | CAN_ID_EXT;//��չ֡ID��λ
	 CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0xffff;
	 CAN_FilterInitStructure.CAN_FilterMaskIdLow=0xffff;				//ID������ȫƥ��
	 CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_FIFO0; //������0
	 CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;			 //ENABLE;
	 CAN_FilterInit(&CAN_FilterInitStructure);
	 
	 CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);  //��FIFO0�ж�
	 
	 NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	 NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
	 NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;     // �����ȼ�Ϊ0
	 NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;            // �����ȼ�Ϊ0
	 NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	 NVIC_Init(&NVIC_InitStructure);
	 
	  GPIO_SetBits(GPIOA,GPIO_Pin_8);
	 
}

unsigned char Flag_FCW;       //FCW��־
unsigned char Time_HMW;       //��ǰ��ʱ��
unsigned int  Distance_FCW;   //��ǰ������
unsigned char Flag_LDW;       //LDW��־
unsigned int  Distance_LDWL;  //���־���
unsigned int  Distance_LDWR;  //���־���


void Data_Process(void)//���ݴ���
{
  Flag_FCW = Param[3];//FCW
	Time_HMW = Param[4];//��ǰ��ʱ��
	Distance_FCW = ((u16)(Param[6]<<8) + Param[5]);//��ǰ������
	Flag_LDW = Param[7];//LDW
	Distance_LDWL = ((u16)(Param[9]<<8) + Param[8]);//���־���
	Distance_LDWR = ((u16)(Param[11]<<8) + Param[10]);//���־���
}



/*��������:CAN_TX(word CAN_ID)                     
* ��������:CAN���ͺ���
* �������:word CAN_ID
* �������:��
* �� �� ֵ:��
*/
uint8_t CAN_TX(void)
{
  uint16_t i=0;
	uint8_t TransmitMailbox;
	
	TxMessage.ExtId=0x18FF2BF2;					//��չ֡��ʶ��
	TxMessage.RTR=CAN_RTR_DATA;			//��Զ��֡
	TxMessage.IDE=CAN_ID_EXT;				//��չ֡
	TxMessage.DLC=8;								//���ݳ���-8byte
	TxMessage.Data[0]=0x02;				//�豸״̬

	if((Flag_LDW == 0x01) || (Flag_LDW == 0x02))//��ʵ�ߡ�������
	{
	  TxMessage.Data[1] = 0x04;	

		Flag_LDWLamp = 1;
	}
	else if((Flag_LDW == 0x03) || (Flag_LDW == 0x04))//��ʵ�ߡ�������
	{
	  TxMessage.Data[1] = 0x01;	

		Flag_LDWLamp = 1;
	}
  	else
	{
	  TxMessage.Data[1] = 0x00;	//�����ޱ���
	}
	
	switch(Flag_FCW)
	{
		case 0x00:
	            TxMessage.Data[2] = 0x00;	//�ޱ���			       
			        break;
		case 0x06:
	            TxMessage.Data[2] = 0x01;	//����1��	

              Flag_FCWLamp = 1;		
			        break;
		case 0x05:
	            TxMessage.Data[2] = 0x02;	//����2��
	
		          Flag_FCWLamp = 1;		
			        break;	
		case 0x0e:
	            TxMessage.Data[2] = 0x04;	//ǰ���г���ȫ
		
              Flag_FCWLamp = 1;				
			        break;
		case 0x0f:
	            TxMessage.Data[2] = 0x08;	//ǰ���г�,��������ѱ���  
		
              Flag_FCWLamp = 1;			
			        break;		
    default : 
			        TxMessage.Data[2] = 0x00;	//�ޱ���		
              break;			
	}

	if(Flag_FCW == 0x0a)
	{
  	TxMessage.Data[3] = 0x01;       //FPWǰ���ﳵ����	
	}
	else
	{
	  TxMessage.Data[3] = 0x00;       //FPWǰ���ﳵ����ȡ��
	}
	if(Flag_FCW == 0x0b)
	{
  	TxMessage.Data[4] = 0x01;       //FVSMǰ����������
	}
	else
	{
  	TxMessage.Data[4] = 0x00;       //FVSMǰ����������ȡ��	
	}
	TxMessage.Data[5]=Time_HMW;           //��ײʱ��
	TxMessage.Data[6]=0x00;							//����
	TxMessage.Data[7]=CAN_Data7++;		  //Life�ź�
	TransmitMailbox = CAN_Transmit(CAN1,&TxMessage);		//����һ֡���ݷ��ط���������ӱ��	
	while((CAN_TransmitStatus(CAN1, TransmitMailbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;	//�ȴ����ͽ���
	if(i>=0XFFF)return 1;
	return 0;
	if(255==CAN_Data7)	//��ֹLife�������
		{
			CAN_Data7=0;
		}
		
		

}

uint8_t Flag_FirstSartL,Flag_FirstSartR = 0;

//CAN�������ݽ���
//int Serial_Datas(void)
//{
//	
////	if(Flag_CAN_EngineSpeed == 1)             //���ٱ���
////	{
////		 if(Date_Speed/10 > THRESHOLD_SPEED)// 
////			{

////				if(Flag_Speed == 0)
////				{
////					Flag_Speed = 1;
////					if((Flag_VoiceChannel == 1) && (Flag_NightLight == 0))
////					{
////						PowerAmp_OFF();
////					}					
////				}
////				
////				
////			}
////			else
////			{
////				if(Date_Speed == 0)
////				{
////				  Date_Engine = 90;
////				}
////				else
////				{
////          Date_Engine = Date_Speed/2;//����Ҫ���ݳ�������				
////				}
////	
////				
////				
////				if(Flag_Speed == 1)
////					{
////						Flag_Speed = 0;																		
////						if((Flag_VoiceChannel == 1) && (Flag_NightLight == 0) && (Flag_Gears != 3))
////						{
////							PowerAmp_ON();
////						}
////					}
////				
////				
////			}
//			
//			

//	}
//	
//	
//	
//	
//  if(Flag_CAN_Start == 1)              //���Start��ҹ���ź�
//	{
//		
//			if(Date_LockStart == 0x02)//���Start
//			{
//			  SignalInputData &= ~0x04;
//								
//			}
//			else
//			{
//			  SignalInputData |= 0x04;
//			}						
//		
//	}
//	
//	if(Flag_CAN_LEMP == 1)              //����ת���ź�
//	{
//		
//		if(Date_Double_Flash == 0x01)//˫��ʹ�ܣ�����ת��ʧ��
//		{
//		   SignalInputData |= 0x02;
//			 SignalInputData |= 0x01;
//		}
//		else
//		{
//		   if(Date_RightTurn == 0x01)//��ת��
//				{
//					SignalInputData &= ~0x02;			
//				}
//				else 
//				{
//					SignalInputData |= 0x02;
//				}	
//				
//				if(Date_LeftTurn == 0x01)//��ת��
//				{
//					SignalInputData &= ~0x01;						
//				}
//				else 
//				{
//					SignalInputData |= 0x01;
//				}
//		}			
//		

//		
//		
//		if(Date_NightLight == 0x01)//ҹ��
//		{
//		  SignalInputData &= ~0x08;
//		}
//		else
//		{
//		  SignalInputData |= 0x08;
//		}
//		
//	}
//}

//CAN�����жϺ���
void USB_LP_CAN1_RX0_IRQHandler(void)
{

	
	NVIC_DisableIRQ(USB_LP_CAN1_RX0_IRQn);	 //ʧ��CAN1��Ϣ�����ж�
	CAN_ClearITPendingBit(CAN1,CAN_IT_FMP0); //���FIFO0��Ϣ�Һ��жϱ�־λ 
	CAN_Receive(CAN1,CAN_FIFO0, &RxMessage); //��FIFO0�н���������Ϣ������Ϣ�ṹ����
	
				if(RxMessage.ExtId == ENGINESPEED_ID)//����̤��ͳ��ٱ���
			{
				Flag_CAN_EngineSpeed = 1;
				Date_Speed = ((u16)(RxMessage.Data[4]<<8) + RxMessage.Data[3])/10;//����  
	
			}
			else if(RxMessage.ExtId == LEMP_ID)//ҹ�ơ�����ת����ź�
			{
				Flag_CAN_LEMP = 1;
			
				Date_RightTurn = ((RxMessage.Data[0]&0x30)>> 4);//��ת���
				Date_LeftTurn = ((RxMessage.Data[0]&0xc0)>>6);//��ת���
//				Date_Double_Flash = ((RxMessage.Data[3]&0x0c)>>2);//˫����
			}	

	
	CAN_FIFORelease(CAN1,CAN_FIFO0); 		 //Clear interrupt flag
	NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);//Enable CAN1 interrupt of receive
	

	
}