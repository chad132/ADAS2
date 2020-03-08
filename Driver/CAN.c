#include "CAN.h"
#include "USART.h"

CanTxMsg TxMessage;
CanRxMsg RxMessage;

uint8_t CAN_Data7 = 0;
uint8_t Flag_Alarm = 0;//报警标志位
uint8_t Flag_CAN_EngineSpeed= 0;//标志位定义
uint16_t Date_Speed = 0;
uint8_t Date_LeftTurn,Date_RightTurn = 0;
uint8_t Flag_CAN_EngineSpeed,Flag_CAN_LEMP = 0;



unsigned char Flag_FCW;       //FCW标志
unsigned char Time_HMW;       //与前车时间
unsigned int  Distance_FCW;   //与前车距离
unsigned char Flag_LDW;       //LDW标志
unsigned int  Distance_LDWL;  //左轮距离
unsigned int  Distance_LDWR;  //右轮距离

uint8_t Flag_FCWLamp,Flag_LDWLamp;

void CANInit(void)
{
	
	
   GPIO_InitTypeDef  GPIO_InitStructure;
	 CAN_InitTypeDef  CAN_InitStructure;	 
	 CAN_FilterInitTypeDef  CAN_FilterInitStructure;
	 NVIC_InitTypeDef  NVIC_InitStructure;
	
	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA  | RCC_APB2Periph_AFIO, ENABLE);//使能PORTA时钟	                   											 
	 RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);//使能CAN1时钟
	
	 //***CAN-TX  IO Init***//
	 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;		//复用推挽输出
 	 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_Init(GPIOA, &GPIO_InitStructure);
	
		
	 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;		//CAN_S要拉低
 	 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	
	 //***CAN-RX  IO Init***//
	 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;			//浮空输入
	 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
 	 GPIO_Init(GPIOA, &GPIO_InitStructure);
	 
	 CAN_DeInit(CAN1);		//初始设置
	 CAN_StructInit(&CAN_InitStructure);
	 
	 CAN_InitStructure.CAN_TTCM=DISABLE;           // 时间触发通信禁止
 	 CAN_InitStructure.CAN_ABOM=ENABLE;            // 一旦硬件检测到128次11位连续的隐性位，则自动退出离线状态
 	 CAN_InitStructure.CAN_AWUM=DISABLE;           // 自动唤醒模式：清零sleep
 	 CAN_InitStructure.CAN_NART=DISABLE;           // 自动重新传送报文，直到发送成功
 	 CAN_InitStructure.CAN_RFLM=DISABLE;           // FIFO没有锁定，新报文覆盖旧报文
 	 CAN_InitStructure.CAN_TXFP=DISABLE;           // 发送报文优先级确定：标志符确定
 	 CAN_InitStructure.CAN_Mode=CAN_Mode_Normal;   // 模式: CAN_Mode_Normal /CAN_Mode_LoopBack/CAN_Mode_Silent_LoopBack
	 CAN_InitStructure.CAN_SJW=CAN_SJW_1tq;        // SWJ:(0-4)
	 CAN_InitStructure.CAN_BS1=CAN_BS1_4tq;				 // BS1:(0-16)
	 CAN_InitStructure.CAN_BS2=CAN_BS2_3tq;				 // BS2:(0-8)
	 CAN_InitStructure.CAN_Prescaler=18;           // 分频数:([9:0]) CAN波特率={36MHZ/(1+4+3)}/18=250KB
	 CAN_Init(CAN1, &CAN_InitStructure);           // 初始化CAN1
	 
	 /************需要用到三个过滤组**************/
	 
	 CAN_FilterInitStructure.CAN_FilterNumber=0;			//过滤组0,用于过滤油门踏板和车速报文
	 CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdList; //列表模式
	 CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;
	 CAN_FilterInitStructure.CAN_FilterIdHigh=((ENGINESPEED_ID<<3) >>16) &0xffff;				//设置扩展帧ID高位
	 CAN_FilterInitStructure.CAN_FilterIdLow=(uint16_t)(ENGINESPEED_ID<<3) | CAN_ID_EXT;//扩展帧ID低位
	 CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0xffff;
	 CAN_FilterInitStructure.CAN_FilterMaskIdLow=0xffff;				//
	 CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_FIFO0; //缓冲器0
	 CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;			 //ENABLE;
	 CAN_FilterInit(&CAN_FilterInitStructure);
	 
	 CAN_FilterInitStructure.CAN_FilterNumber=1;			//过滤组1，用于Start信号报文
	 CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdList; //列表模式
	 CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;
	 CAN_FilterInitStructure.CAN_FilterIdHigh=((START_ID<<3) >>16) &0xffff;				//设置扩展帧ID高位
	 CAN_FilterInitStructure.CAN_FilterIdLow=(uint16_t)(START_ID<<3) | CAN_ID_EXT;//扩展帧ID低位
	 CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0xffff;
	 CAN_FilterInitStructure.CAN_FilterMaskIdLow=0xffff;				//ID不必完全匹配
	 CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_FIFO0; //缓冲器0
	 CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;			 //ENABLE;
	 CAN_FilterInit(&CAN_FilterInitStructure);
	 
	 CAN_FilterInitStructure.CAN_FilterNumber=2;			//过滤组2，用于过滤左右转向灯信号报文
	 CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdList; //列表模式
	 CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;
	 CAN_FilterInitStructure.CAN_FilterIdHigh=((LEMP_ID<<3) >>16) &0xffff;				//设置扩展帧ID高位
	 CAN_FilterInitStructure.CAN_FilterIdLow=(uint16_t)(LEMP_ID<<3) | CAN_ID_EXT;//扩展帧ID低位
	 CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0xffff;
	 CAN_FilterInitStructure.CAN_FilterMaskIdLow=0xffff;				//ID不必完全匹配
	 CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_FIFO0; //缓冲器0
	 CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;			 //ENABLE;
	 CAN_FilterInit(&CAN_FilterInitStructure);
	 
	 CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);  //打开FIFO0中断
	 
	 NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	 NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
	 NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;     // 主优先级为0
	 NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;            // 次优先级为0
	 NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	 NVIC_Init(&NVIC_InitStructure);
	 
	  GPIO_SetBits(GPIOA,GPIO_Pin_8);
	 
}

unsigned char Flag_FCW;       //FCW标志
unsigned char Time_HMW;       //与前车时间
unsigned int  Distance_FCW;   //与前车距离
unsigned char Flag_LDW;       //LDW标志
unsigned int  Distance_LDWL;  //左轮距离
unsigned int  Distance_LDWR;  //右轮距离


void Data_Process(void)//数据处理
{
  Flag_FCW = Param[3];//FCW
	Time_HMW = Param[4];//与前车时间
	Distance_FCW = ((u16)(Param[6]<<8) + Param[5]);//与前车距离
	Flag_LDW = Param[7];//LDW
	Distance_LDWL = ((u16)(Param[9]<<8) + Param[8]);//左轮距离
	Distance_LDWR = ((u16)(Param[11]<<8) + Param[10]);//右轮距离
}



/*函数名称:CAN_TX(word CAN_ID)                     
* 函数介绍:CAN发送函数
* 输入参数:word CAN_ID
* 输出参数:无
* 返 回 值:无
*/
uint8_t CAN_TX(void)
{
  uint16_t i=0;
	uint8_t TransmitMailbox;
	
	TxMessage.ExtId=0x18FF2BF2;					//扩展帧标识符
	TxMessage.RTR=CAN_RTR_DATA;			//非远程帧
	TxMessage.IDE=CAN_ID_EXT;				//扩展帧
	TxMessage.DLC=8;								//数据长度-8byte
	TxMessage.Data[0]=0x02;				//设备状态

	if((Flag_LDW == 0x01) || (Flag_LDW == 0x02))//左实线、左虚线
	{
	  TxMessage.Data[1] = 0x04;	

		Flag_LDWLamp = 1;
	}
	else if((Flag_LDW == 0x03) || (Flag_LDW == 0x04))//右实线、右虚线
	{
	  TxMessage.Data[1] = 0x01;	

		Flag_LDWLamp = 1;
	}
  	else
	{
	  TxMessage.Data[1] = 0x00;	//其它无报警
	}
	
	switch(Flag_FCW)
	{
		case 0x00:
	            TxMessage.Data[2] = 0x00;	//无报警			       
			        break;
		case 0x06:
	            TxMessage.Data[2] = 0x01;	//报警1低	

              Flag_FCWLamp = 1;		
			        break;
		case 0x05:
	            TxMessage.Data[2] = 0x02;	//报警2高
	
		          Flag_FCWLamp = 1;		
			        break;	
		case 0x0e:
	            TxMessage.Data[2] = 0x04;	//前方有车安全
		
              Flag_FCWLamp = 1;				
			        break;
		case 0x0f:
	            TxMessage.Data[2] = 0x08;	//前方有车,车距近，已报警  
		
              Flag_FCWLamp = 1;			
			        break;		
    default : 
			        TxMessage.Data[2] = 0x00;	//无报警		
              break;			
	}

	if(Flag_FCW == 0x0a)
	{
  	TxMessage.Data[3] = 0x01;       //FPW前车溜车警告	
	}
	else
	{
	  TxMessage.Data[3] = 0x00;       //FPW前车溜车警告取消
	}
	if(Flag_FCW == 0x0b)
	{
  	TxMessage.Data[4] = 0x01;       //FVSM前车启动提醒
	}
	else
	{
  	TxMessage.Data[4] = 0x00;       //FVSM前车启动提醒取消	
	}
	TxMessage.Data[5]=Time_HMW;           //碰撞时间
	TxMessage.Data[6]=0x00;							//保留
	TxMessage.Data[7]=CAN_Data7++;		  //Life信号
	TransmitMailbox = CAN_Transmit(CAN1,&TxMessage);		//发送一帧数据返回发送邮箱盒子标号	
	while((CAN_TransmitStatus(CAN1, TransmitMailbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;	//等待发送结束
	if(i>=0XFFF)return 1;
	return 0;
	if(255==CAN_Data7)	//防止Life数据溢出
		{
			CAN_Data7=0;
		}
		
		

}

uint8_t Flag_FirstSartL,Flag_FirstSartR = 0;

//CAN接收数据解析
//int Serial_Datas(void)
//{
//	
////	if(Flag_CAN_EngineSpeed == 1)             //车速报文
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
////          Date_Engine = Date_Speed/2;//油门要根据车速来调				
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
//  if(Flag_CAN_Start == 1)              //点火Start和夜灯信号
//	{
//		
//			if(Date_LockStart == 0x02)//点火Start
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
//	if(Flag_CAN_LEMP == 1)              //左右转弯信号
//	{
//		
//		if(Date_Double_Flash == 0x01)//双闪使能，左右转弯失能
//		{
//		   SignalInputData |= 0x02;
//			 SignalInputData |= 0x01;
//		}
//		else
//		{
//		   if(Date_RightTurn == 0x01)//右转弯
//				{
//					SignalInputData &= ~0x02;			
//				}
//				else 
//				{
//					SignalInputData |= 0x02;
//				}	
//				
//				if(Date_LeftTurn == 0x01)//左转弯
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
//		if(Date_NightLight == 0x01)//夜灯
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

//CAN接收中断函数
void USB_LP_CAN1_RX0_IRQHandler(void)
{

	
	NVIC_DisableIRQ(USB_LP_CAN1_RX0_IRQn);	 //失能CAN1消息接收中断
	CAN_ClearITPendingBit(CAN1,CAN_IT_FMP0); //清除FIFO0消息挂号中断标志位 
	CAN_Receive(CAN1,CAN_FIFO0, &RxMessage); //将FIFO0中接收数据信息存入消息结构体中
	
				if(RxMessage.ExtId == ENGINESPEED_ID)//油门踏板和车速报文
			{
				Flag_CAN_EngineSpeed = 1;
				Date_Speed = ((u16)(RxMessage.Data[4]<<8) + RxMessage.Data[3])/10;//车速  
	
			}
			else if(RxMessage.ExtId == LEMP_ID)//夜灯、左右转向灯信号
			{
				Flag_CAN_LEMP = 1;
			
				Date_RightTurn = ((RxMessage.Data[0]&0x30)>> 4);//右转向灯
				Date_LeftTurn = ((RxMessage.Data[0]&0xc0)>>6);//左转向灯
//				Date_Double_Flash = ((RxMessage.Data[3]&0x0c)>>2);//双闪灯
			}	

	
	CAN_FIFORelease(CAN1,CAN_FIFO0); 		 //Clear interrupt flag
	NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);//Enable CAN1 interrupt of receive
	

	
}