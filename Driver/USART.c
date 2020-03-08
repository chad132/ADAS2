#include "USART.h"
#include "CAN.h"

uint8_t Voice_Num = 0;
uint8_t Data_CAR[13] = {0};   //�����ϱ����ݻ���
uint8_t Data_Calibration [27] = {0};   //�궨���ݻ���
	


typedef struct 
{
  unsigned char Flag_FCW;       //FCW��־
	unsigned char Time_HMW;       //��ǰ��ʱ��
	unsigned int  Distance_FCW;   //��ǰ������
	unsigned char Flag_LDW;       //LDW��־
	unsigned int  Distance_LDWL;  //���־���
  unsigned int  Distance_LDWR;  //���־���	
	
	
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


//�������ݴ���
void Data_CAR_Process(void)//���ݴ���
{
	static uint8_t Num_Data2;//֡���
	uint8_t Data_CAR_CRC[7] = {0};//�����ϱ�����CRC����
	uint16_t Sum_Data_CAR_CRC = 0;
	uint8_t Num_Data3 = 0;
	
  Data_CAR[Num_Data3++] = 0x5E;//֡ͷ  0
	Num_Data2++;
	if(Num_Data2 == 0x5e)
	{
	  	Data_CAR[Num_Data3++] = (Num_Data2+1);//��� 1
		  Data_CAR[Num_Data3++] = 0x01;	        //2
	}
	else if(Num_Data2 == 0x5f)
	{
	 	  Data_CAR[Num_Data3++] = Num_Data2;//��� 1
		  Data_CAR[Num_Data3++] = 0x02;//2
	}
	else
	{
	   	Data_CAR[Num_Data3++] = Num_Data2;//��� 1
	}
	
	
	Data_CAR[Num_Data3++] = 0x40;//����                     2   3
	Data_CAR_CRC[0] = 0x40;
	Data_CAR[Num_Data3++] = 0x04;//���ݳ��ȵ��ֽ�           3   4
	Data_CAR_CRC[1] = 0x04;	
	Data_CAR[Num_Data3++] = 0x00;//���ݳ��ȸ��ֽ�           4   5
	Data_CAR_CRC[2] = 0x00;	
	Data_CAR[Num_Data3++] = (uint8_t)Date_Speed;//���ٵ��ֽ�               5   6//(uint8_t)Date_Speed
	Data_CAR_CRC[3] = (uint8_t)Date_Speed;	                                        //
	Data_CAR[Num_Data3++] = (uint8_t)(Date_Speed>>8);//���ٸ��ֽ�               6   7    //(uint8_t)(Date_Speed>>8)
	Data_CAR_CRC[4] = (uint8_t)(Date_Speed>>8);	                                         //(uint8_t)(Date_Speed>>8)
	Data_CAR[Num_Data3++] = (0x18 & ((Date_LeftTurn<<3) | (Date_RightTurn<<4)));//����Ӳ���ź�������ֽ�    7   8  (0x18 & ((Date_LeftTurn<<3) || (Date_RightTurn<<4)))
	Data_CAR_CRC[5] = (0x18 & ((Date_LeftTurn<<3) | (Date_RightTurn<<4)));	
	Data_CAR[Num_Data3++] = 0x00;//����Ӳ���ź�������ֽ�    8   9
	Data_CAR_CRC[6] = 0x00;	
	Sum_Data_CAR_CRC = MakeCRC(Data_CAR_CRC,7); //CRC����
	Data_CAR[Num_Data3++] = (uint8_t)Sum_Data_CAR_CRC;//CRC���ֽ�                9   10
	Data_CAR[Num_Data3++] = (uint8_t)(Sum_Data_CAR_CRC>>8);//CRC���ֽ�                10  11
	Data_CAR[Num_Data3] = 0x5E;//֡β                       11  12
	
  if((Num_Data2 == 0x5e) || (Num_Data2 == 0x5f))
	{
	  	USART1_SendArry(Data_CAR,13);
	}
	else
	{
	  USART1_SendArry(Data_CAR,12);
	}



	
}


//�����궨���ݴ�������
void Data_Calibration_Process(void)//���ݴ���
{
	static uint8_t Num_Data2;//֡���
	uint8_t Data_Calibration_CRC[21] = {0};//�궨����CRC����
	uint16_t Sum_Data_Calibration_CRC = 0;
	uint8_t Num_Data3 = 0;
	
  Data_Calibration[Num_Data3++] = 0x5E;//֡ͷ  0
	Num_Data2++;
	if(Num_Data2 == 0x5e)
	{
	  	Data_Calibration[Num_Data3++] = (Num_Data2+1);//��� 1
		  Data_Calibration[Num_Data3++] = 0x01;	        //2
	}
	else if(Num_Data2 == 0x5f)
	{
	 	  Data_Calibration[Num_Data3++] = Num_Data2;//��� 1
		  Data_Calibration[Num_Data3++] = 0x02;//2
	}
	else
	{
	   	Data_Calibration[Num_Data3++] = Num_Data2;//��� 1
	}
	
	
	Data_Calibration[Num_Data3++] = 0x16;//����                     2   3
	Data_Calibration_CRC[0] = 0x16;
	Data_Calibration[Num_Data3++] = 0x12;//���ݳ��ȵ��ֽ�           3   4  ���ȣ�18
	Data_Calibration_CRC[1] = 0x12;	
	Data_Calibration[Num_Data3++] = 0x00;//���ݳ��ȸ��ֽ�           4   5
	Data_Calibration_CRC[2] = 0x00;
	
	Data_Calibration[Num_Data3++] = 0x01;//������ȵ��ֽ�               5   6
	Data_Calibration_CRC[3] = 0x01;	
	Data_Calibration[Num_Data3++] = 0x02;//������ȸ��ֽ�              6   7
	Data_Calibration_CRC[4] = 0x02;	
	Data_Calibration[Num_Data3++] = 0x03;//�����߶ȵ��ֽ�    7   8
	Data_Calibration_CRC[5] = 0x03;	
	Data_Calibration[Num_Data3++] = 0x04;//�����߶ȸ��ֽ�    8   9
	Data_Calibration_CRC[6] = 0x04;	
	Data_Calibration[Num_Data3++] = 0x05;//�������ȵ��ֽ�    7   8
	Data_Calibration_CRC[7] = 0x05;	
	Data_Calibration[Num_Data3++] = 0x06;//�������ȸ��ֽ�    8   9
	Data_Calibration_CRC[8] = 0x06;	
	
	Data_Calibration[Num_Data3++] = 0x07;//����복��������ֽ�               5   6
	Data_Calibration_CRC[9] = 0x07;	
	Data_Calibration[Num_Data3++] = 0x08;//����복��������ֽ�              6   7
	Data_Calibration_CRC[10] = 0x08;	
	Data_Calibration[Num_Data3++] = 0x09;//����복ͷ������ֽ�    7   8
	Data_Calibration_CRC[11] = 0x09;	
	Data_Calibration[Num_Data3++] = 0x0a;//����복ͷ������ֽ�    8   9
	Data_Calibration_CRC[12] = 0x0a;	
	Data_Calibration[Num_Data3++] = 0x0b;//�������������ֽ�   7   8
	Data_Calibration_CRC[13] = 0x0b;	
	Data_Calibration[Num_Data3++] = 0x0c;//�������������ֽ�    8   9
	Data_Calibration_CRC[14] = 0x0c;	
	

	Data_Calibration[Num_Data3++] = 0x0d;//X����ֽ�               5   6
	Data_Calibration_CRC[15] = 0x0d;	
	Data_Calibration[Num_Data3++] = 0x0e;//X����ֽ�              6   7
	Data_Calibration_CRC[16] = 0x0e;	
	Data_Calibration[Num_Data3++] = 0x0f;//Y����ֽ�     7   8
	Data_Calibration_CRC[17] = 0x0f;	
	Data_Calibration[Num_Data3++] = 0x10;//Y����ֽ�     8   9
	Data_Calibration_CRC[18] = 0x10;	
	Data_Calibration[Num_Data3++] = 0x11;//Z����ֽ�    7   8
	Data_Calibration_CRC[19] = 0x11;	
	Data_Calibration[Num_Data3++] = 0x12;//Z����ֽ�     8   9
	Data_Calibration_CRC[20] = 0x12;



	
	Sum_Data_Calibration_CRC = MakeCRC(Data_Calibration_CRC,7); //CRC����
	Data_Calibration[Num_Data3++] = (uint8_t)Sum_Data_Calibration_CRC;//CRC���ֽ�                9   10
	Data_Calibration[Num_Data3++] = (uint8_t)(Sum_Data_Calibration_CRC) << 8;//CRC���ֽ�                10  11
	Data_Calibration[Num_Data3] = 0x5E;//֡β                       11  12
	
  if((Num_Data2 == 0x5e) || (Num_Data2 == 0x5f))
	{
	  	USART1_SendArry(Data_Calibration,27);
	}
	else
	{
	  USART1_SendArry(Data_Calibration,26);
	}



	
}

//���ݷ���
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




//CRC����
unsigned short MakeCRC(unsigned char * ptr,unsigned char len)//CRC����
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
				if( 0x5e == data)//֡ͷ
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
				if( 0x5e != data)//��ţ����˵���β����0x5e������
				{
					rxMessage1.state = 0x02;
				}
        break;
		case 0x02:
			if( 0x38 == data)//����,   0x38:������Ϣ
				{
					rxMessage1.state = 0x03;
					Param[0] = data;
				}
				rxMessage1.count1 = 2;//���ݳ��ȼ���λ��ֵΪ2
        break;	
		case 0x03:
			   *((char *)(&rxMessage1.Length) + (--rxMessage1.count1)) = data;	
				if (rxMessage1.count1 == 0)	
				{
			     num = (uint8_t)(rxMessage1.Length>>8);//���ݳ��ȸ�ֵ   					
					rxMessage1.state = 0x04;			  
				}					
        Param[1] = 0x09;
				Param[2] = 0x00;
        break;				
		case 0x04://����
//        *((char *)(&rxMessage1.payload) + (--num)) = data;
//		       if(num == 0)
//					 {
//						 Sum_CRC = MakeCRC(Param,9);    
//						 rxMessage1.state = 0x05;		
//						 rxMessage1.count1 = 2;//���ݳ��ȼ���λ��ֵΪ2						 
//					 }
		
		
         Param[data_num++] = data;
          		          		
				if ((num+3) == data_num)
				{
           Sum_CRC = MakeCRC(Param,12);    
					 rxMessage1.state = 0x05;		
           rxMessage1.count1 = 2;//���ݳ��ȼ���λ��ֵΪ2	
           data_CRC = 0;					
				}					


        break;
  	case 0x05://CRCУ��
			   *((char *)(&rxMessage1.Data_CRC) + (data_CRC++)) = data;	
		     
//				if (rxMessage1.Data_CRC == Sum_CRC)		
		      if(rxMessage1.count1 == data_CRC)
		        rxMessage1.state = 0x06;

        break;	
				
		case 0x06:
        if( 0x5e == data)//֡β
				{
					rxMessage1.state = 0x00;
//					GPIOA->ODR ^= GPIO_Pin_6;//��ɫLED��ת
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