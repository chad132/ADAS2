#include "USART.h"
#include "CAN.h"

uint8_t Voice_Num = 0;
uint8_t Data_CAR[13] = {0};   //车辆上报数据缓存
uint8_t Data_Calibration [27] = {0};   //标定数据缓存
	


typedef struct 
{
  unsigned char Flag_FCW;       //FCW标志
	unsigned char Time_HMW;       //与前车时间
	unsigned int  Distance_FCW;   //与前车距离
	unsigned char Flag_LDW;       //LDW标志
	unsigned int  Distance_LDWL;  //左轮距离
  unsigned int  Distance_LDWR;  //右轮距离	
	
	
} SeriAlarmStruct_t;

typedef struct 
{
   
    unsigned char state;
	  unsigned char RxCK_A;
    unsigned char count;
	  unsigned char count1;
	  unsigned int  Length; 
	  unsigned int  Data_CRC; 
		SeriAlarmStruct_t payload;
		
} SeriMessgeStruct_t;

SeriMessgeStruct_t  rxMessage1;


//USART1 PA.9 PA.10
void USART1_Init(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 , ENABLE);


  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);    

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	  
  
		/* Enable the USART2 Interrupt */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;


	USART_Init(USART1, &USART_InitStructure); 
	//USART_ITConfig(USART1, USART_IT_TXE, DISABLE);        
  USART_ClearFlag(USART1,USART_FLAG_TC);
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);	

	
  USART_Cmd(USART1, ENABLE);

}


//void USART2_TxHigh(void)
//{
//  GPIO_SetBits(GPIOA,GPIO_Pin_2);
//}

//void USART2_TxLow(void)
//{

//	USART_Cmd(USART2, DISABLE);
//		
//  GPIO_ResetBits(GPIOA,GPIO_Pin_2);
//}


void Txd(unsigned char byte)
{
	USART_SendData(USART1, (unsigned char) byte); 
	while (!(USART1->SR & USART_FLAG_TXE));
} 

void USART1_SendArry(unsigned char *pdata, unsigned char len)
{ 
   unsigned char i ;
   
   for(i = 0 ; i < len ; i++)
   { 
      USART_SendData(USART1, pdata[i]); 
      while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
    } 
} 


//车辆数据处理
void Data_CAR_Process(void)//数据处理
{
	static uint8_t Num_Data2;//帧序号
	uint8_t Data_CAR_CRC[7] = {0};//车辆上报数据CRC缓存
	uint16_t Sum_Data_CAR_CRC = 0;
	uint8_t Num_Data3 = 0;
	
  Data_CAR[Num_Data3++] = 0x5E;//帧头  0
	Num_Data2++;
	if(Num_Data2 == 0x5e)
	{
	  	Data_CAR[Num_Data3++] = (Num_Data2+1);//序号 1
		  Data_CAR[Num_Data3++] = 0x01;	        //2
	}
	else if(Num_Data2 == 0x5f)
	{
	 	  Data_CAR[Num_Data3++] = Num_Data2;//序号 1
		  Data_CAR[Num_Data3++] = 0x02;//2
	}
	else
	{
	   	Data_CAR[Num_Data3++] = Num_Data2;//序号 1
	}
	
	
	Data_CAR[Num_Data3++] = 0x40;//命令                     2   3
	Data_CAR_CRC[0] = 0x40;
	Data_CAR[Num_Data3++] = 0x04;//数据长度低字节           3   4
	Data_CAR_CRC[1] = 0x04;	
	Data_CAR[Num_Data3++] = 0x00;//数据长度高字节           4   5
	Data_CAR_CRC[2] = 0x00;	
	Data_CAR[Num_Data3++] = (uint8_t)Date_Speed;//车速低字节               5   6//(uint8_t)Date_Speed
	Data_CAR_CRC[3] = (uint8_t)Date_Speed;	                                        //
	Data_CAR[Num_Data3++] = (uint8_t)(Date_Speed>>8);//车速高字节               6   7    //(uint8_t)(Date_Speed>>8)
	Data_CAR_CRC[4] = (uint8_t)(Date_Speed>>8);	                                         //(uint8_t)(Date_Speed>>8)
	Data_CAR[Num_Data3++] = (0x18 & ((Date_LeftTurn<<3) | (Date_RightTurn<<4)));//车辆硬线信号输入低字节    7   8  (0x18 & ((Date_LeftTurn<<3) || (Date_RightTurn<<4)))
	Data_CAR_CRC[5] = (0x18 & ((Date_LeftTurn<<3) | (Date_RightTurn<<4)));	
	Data_CAR[Num_Data3++] = 0x00;//车辆硬线信号输入高字节    8   9
	Data_CAR_CRC[6] = 0x00;	
	Sum_Data_CAR_CRC = MakeCRC(Data_CAR_CRC,7); //CRC计算
	Data_CAR[Num_Data3++] = (uint8_t)Sum_Data_CAR_CRC;//CRC低字节                9   10
	Data_CAR[Num_Data3++] = (uint8_t)(Sum_Data_CAR_CRC>>8);//CRC高字节                10  11
	Data_CAR[Num_Data3] = 0x5E;//帧尾                       11  12
	
  if((Num_Data2 == 0x5e) || (Num_Data2 == 0x5f))
	{
	  	USART1_SendArry(Data_CAR,13);
	}
	else
	{
	  USART1_SendArry(Data_CAR,12);
	}



	
}


//参数标定数据处理并发送
void Data_Calibration_Process(void)//数据处理
{
	static uint8_t Num_Data2;//帧序号
	uint8_t Data_Calibration_CRC[21] = {0};//标定参数CRC缓存
	uint16_t Sum_Data_Calibration_CRC = 0;
	uint8_t Num_Data3 = 0;
	
  Data_Calibration[Num_Data3++] = 0x5E;//帧头  0
	Num_Data2++;
	if(Num_Data2 == 0x5e)
	{
	  	Data_Calibration[Num_Data3++] = (Num_Data2+1);//序号 1
		  Data_Calibration[Num_Data3++] = 0x01;	        //2
	}
	else if(Num_Data2 == 0x5f)
	{
	 	  Data_Calibration[Num_Data3++] = Num_Data2;//序号 1
		  Data_Calibration[Num_Data3++] = 0x02;//2
	}
	else
	{
	   	Data_Calibration[Num_Data3++] = Num_Data2;//序号 1
	}
	
	
	Data_Calibration[Num_Data3++] = 0x16;//命令                     2   3
	Data_Calibration_CRC[0] = 0x16;
	Data_Calibration[Num_Data3++] = 0x12;//数据长度低字节           3   4  长度：18
	Data_Calibration_CRC[1] = 0x12;	
	Data_Calibration[Num_Data3++] = 0x00;//数据长度高字节           4   5
	Data_Calibration_CRC[2] = 0x00;
	
	Data_Calibration[Num_Data3++] = 0x01;//车辆宽度低字节               5   6
	Data_Calibration_CRC[3] = 0x01;	
	Data_Calibration[Num_Data3++] = 0x02;//车辆宽度高字节              6   7
	Data_Calibration_CRC[4] = 0x02;	
	Data_Calibration[Num_Data3++] = 0x03;//车辆高度低字节    7   8
	Data_Calibration_CRC[5] = 0x03;	
	Data_Calibration[Num_Data3++] = 0x04;//车辆高度高字节    8   9
	Data_Calibration_CRC[6] = 0x04;	
	Data_Calibration[Num_Data3++] = 0x05;//车辆长度低字节    7   8
	Data_Calibration_CRC[7] = 0x05;	
	Data_Calibration[Num_Data3++] = 0x06;//车辆长度高字节    8   9
	Data_Calibration_CRC[8] = 0x06;	
	
	Data_Calibration[Num_Data3++] = 0x07;//相机与车辆距离低字节               5   6
	Data_Calibration_CRC[9] = 0x07;	
	Data_Calibration[Num_Data3++] = 0x08;//相机与车辆距离低字节              6   7
	Data_Calibration_CRC[10] = 0x08;	
	Data_Calibration[Num_Data3++] = 0x09;//相机与车头距离低字节    7   8
	Data_Calibration_CRC[11] = 0x09;	
	Data_Calibration[Num_Data3++] = 0x0a;//相机与车头距离低字节    8   9
	Data_Calibration_CRC[12] = 0x0a;	
	Data_Calibration[Num_Data3++] = 0x0b;//相机与地面距离低字节   7   8
	Data_Calibration_CRC[13] = 0x0b;	
	Data_Calibration[Num_Data3++] = 0x0c;//相机与地面距离低字节    8   9
	Data_Calibration_CRC[14] = 0x0c;	
	

	Data_Calibration[Num_Data3++] = 0x0d;//X轴低字节               5   6
	Data_Calibration_CRC[15] = 0x0d;	
	Data_Calibration[Num_Data3++] = 0x0e;//X轴高字节              6   7
	Data_Calibration_CRC[16] = 0x0e;	
	Data_Calibration[Num_Data3++] = 0x0f;//Y轴低字节     7   8
	Data_Calibration_CRC[17] = 0x0f;	
	Data_Calibration[Num_Data3++] = 0x10;//Y轴高字节     8   9
	Data_Calibration_CRC[18] = 0x10;	
	Data_Calibration[Num_Data3++] = 0x11;//Z轴低字节    7   8
	Data_Calibration_CRC[19] = 0x11;	
	Data_Calibration[Num_Data3++] = 0x12;//Z轴高字节     8   9
	Data_Calibration_CRC[20] = 0x12;



	
	Sum_Data_Calibration_CRC = MakeCRC(Data_Calibration_CRC,7); //CRC计算
	Data_Calibration[Num_Data3++] = (uint8_t)Sum_Data_Calibration_CRC;//CRC低字节                9   10
	Data_Calibration[Num_Data3++] = (uint8_t)(Sum_Data_Calibration_CRC) << 8;//CRC高字节                10  11
	Data_Calibration[Num_Data3] = 0x5E;//帧尾                       11  12
	
  if((Num_Data2 == 0x5e) || (Num_Data2 == 0x5f))
	{
	  	USART1_SendArry(Data_Calibration,27);
	}
	else
	{
	  USART1_SendArry(Data_Calibration,26);
	}



	
}

//数据发送
//void Data_CAR_Send(void)
//{
//  Data_CAR_Process();
//	USART1_SendArry(Data_CAR,13);
//	
//}

static void RxChecksumReset1(void) 
{
    rxMessage1.RxCK_A= 0x5f;
}

static void RxChecksum1(unsigned char c) 
{
    rxMessage1.RxCK_A += c;

}




//CRC计算
unsigned short MakeCRC(unsigned char * ptr,unsigned char len)//CRC计算
{
	unsigned char num1,num2;
	unsigned short data_crc = 0xFFff;
	for (num1=0; num1<len; num1++)
		{
				data_crc ^= ptr[num1];
				for (num2=0; num2<8; num2++)
			{
				if (data_crc & 0x0001)
					 data_crc = (data_crc >> 1) ^ 0x8408;
				else
						data_crc = (data_crc >>1);
			}
	  }
	return data_crc;
	

	
	
	
}


uint8_t Param[12] = {0};
unsigned short Sum_CRC = 0 ;
int Serial_Datas(unsigned char data)
{
	static uint16_t num;
	static uint8_t data_num,data_CRC;
  switch(rxMessage1.state)
	{
		case 0x00:
				if( 0x5e == data)//帧头
				{
					rxMessage1.state = 0x01;
				}
				rxMessage1.count1 = 2;
				rxMessage1.Length = 0;
				num = 0;
				data_num = 3;
				RxChecksumReset1() ;
		    break;
		case 0x01:
				if( 0x5e != data)//序号，过滤掉首尾都是0x5e的问题
				{
					rxMessage1.state = 0x02;
				}
        break;
		case 0x02:
			if( 0x38 == data)//命令,   0x38:报警信息
				{
					rxMessage1.state = 0x03;
					Param[0] = data;
				}
				rxMessage1.count1 = 2;//数据长度计数位赋值为2
        break;	
		case 0x03:
			   *((char *)(&rxMessage1.Length) + (--rxMessage1.count1)) = data;	
				if (rxMessage1.count1 == 0)	
				{
			     num = (uint8_t)(rxMessage1.Length>>8);//数据长度赋值   					
					rxMessage1.state = 0x04;			  
				}					
        Param[1] = 0x09;
				Param[2] = 0x00;
        break;				
		case 0x04://数据
//        *((char *)(&rxMessage1.payload) + (--num)) = data;
//		       if(num == 0)
//					 {
//						 Sum_CRC = MakeCRC(Param,9);    
//						 rxMessage1.state = 0x05;		
//						 rxMessage1.count1 = 2;//数据长度计数位赋值为2						 
//					 }
		
		
         Param[data_num++] = data;
          		          		
				if ((num+3) == data_num)
				{
           Sum_CRC = MakeCRC(Param,12);    
					 rxMessage1.state = 0x05;		
           rxMessage1.count1 = 2;//数据长度计数位赋值为2	
           data_CRC = 0;					
				}					


        break;
  	case 0x05://CRC校验
			   *((char *)(&rxMessage1.Data_CRC) + (data_CRC++)) = data;	
		     
//				if (rxMessage1.Data_CRC == Sum_CRC)		
		      if(rxMessage1.count1 == data_CRC)
		        rxMessage1.state = 0x06;

        break;	
				
		case 0x06:
        if( 0x5e == data)//帧尾
				{
					rxMessage1.state = 0x00;
//					GPIOA->ODR ^= GPIO_Pin_6;//红色LED翻转
				}
        break;				
    default:
				return -1;
			break;				 
	}
	
	return 0;
}






void USART1_IRQHandler(void)
{
	 uint8_t data = 0;
	
  if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
  {
   data=USART_ReceiveData(USART1);
	 //USART_SendData(USART1,data);
	 Serial_Datas(data);
		
	 USART_ClearITPendingBit(USART1, USART_IT_RXNE);
  }
	
	
}