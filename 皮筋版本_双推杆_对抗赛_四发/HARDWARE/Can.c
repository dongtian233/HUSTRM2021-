/*---------------------------------------------------------------------------*/
/** 
  **@brief ���CANͨ������
  **
  **/
/*---------------------------------------------------------------------------*/


#include "main.h"

u8	Gyro_Initial_Flag;
u8 _6020Flag=0;
float yaw_angle;
float gz;
rmc620_t Gimbal_Motor[MotorNum];//[0]����[1]����[2]yaw//[3]pitch

//extern PID	Pid_Gimbal_Pos_Yaw;

/**
  * @brief  ����CAN1
  * @param  None
  * @retval None
  */
void can1_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	CAN_InitTypeDef        CAN_InitStructure;
	CAN_FilterInitTypeDef  CAN_FilterInitStructure;

	/* ��GPIOʱ�ӡ�AFIOʱ�ӣ�CANʱ�� */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);


	/* CAN1 RX PB8 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	/* CAN1 TX PB9 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_PinRemapConfig(GPIO_Remap1_CAN1,ENABLE);  // CAN1 remap

	/* CAN1 Enabling interrupt */	
	NVIC_Config(CAN1_RX0_IRQn, 0, 0);									
	NVIC_Config(CAN1_TX_IRQn, 1, 0);
	
	/* CAN  BaudRate = RCC_APB1PeriphClock/(CAN_SJW+CAN_BS1+CAN_BS2)/CAN_Prescaler */
	CAN_DeInit(CAN1);
	CAN_StructInit(&CAN_InitStructure);   

	CAN_InitStructure.CAN_TTCM=DISABLE;
	CAN_InitStructure.CAN_ABOM=ENABLE;
	CAN_InitStructure.CAN_AWUM=ENABLE;
	CAN_InitStructure.CAN_NART=ENABLE;
	CAN_InitStructure.CAN_RFLM=DISABLE;
	CAN_InitStructure.CAN_TXFP=ENABLE;
	CAN_InitStructure.CAN_Mode=CAN_Mode_Normal;   
	CAN_InitStructure.CAN_SJW=CAN_SJW_1tq;
	CAN_InitStructure.CAN_BS1=CAN_BS1_5tq;  
	CAN_InitStructure.CAN_BS2=CAN_BS2_3tq;	
	CAN_InitStructure.CAN_Prescaler=4;

	CAN_Init(CAN1,&CAN_InitStructure);	// CAN1											

	CAN_FilterInitStructure.CAN_FilterNumber=0;	 
	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;	 // ��ʶ������λģʽ
	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;   // 32λ������
	CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;			// ��������ʶ��
	CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;				
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;		// ���������α�ʶ��
	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_FIFO0;	 // FIFO0ָ�������
	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;
	CAN_FilterInit(&CAN_FilterInitStructure);
//	CAN1->IER = 0x03;

    CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);
    CAN_ITConfig(CAN1,CAN_IT_TME,ENABLE);
}


void CAN1_TX_IRQHandler(void)
{
	if(CAN_GetITStatus(CAN1,CAN_IT_TME)!=0)
	{
		
		CAN_ClearITPendingBit(CAN1,CAN_IT_TME);
	}
	
}
/**
  * @brief  CAN1�����ж�
  * @param  None
  * @retval None
  */
void CAN1_RX0_IRQHandler(void)
{
	CanRxMsg rx_message;
	if(CAN_GetITStatus(CAN1,CAN_IT_FMP0)!= RESET) 
	{
		CAN_Receive(CAN1, CAN_FIFO0, &rx_message);
		switch(rx_message.StdId)
    {
            case 0x201://PushPull1
                Gimbal_Motor[0].angle = (rx_message.Data[0] << 8) | rx_message.Data[1];
                Gimbal_Motor[0].speed = (rx_message.Data[2] << 8) | rx_message.Data[3];
            break;
						
						case 0x202://PushPull2
                Gimbal_Motor[1].angle = (rx_message.Data[0] << 8) | rx_message.Data[1];
                Gimbal_Motor[1].speed = (rx_message.Data[2] << 8) | rx_message.Data[3];
            break;
          
            case 0x203://recharge
                Gimbal_Motor[2].angle = (rx_message.Data[0] << 8) | rx_message.Data[1];
                Gimbal_Motor[2].speed = (rx_message.Data[2] << 8) | rx_message.Data[3];
            break;    
						
						default:
						break;
		}
		CAN_ClearITPendingBit(CAN1,CAN_IT_FMP0);
		Discnect_Flag.Can1_Cnt=0;
		Discnect_Flag.Can1_Connect=0;
	}
}	
/**
  * @brief  CAN1��������,abcd�ڴ������޷� -6000 ~ 0 ~ 6000��efgh�ڴ���ѹ�޷�10000
  * @param  ��ʶ��0x200
						a��0x201��������
            b��0x202��������
            c��0x203��������
            d��0x204��������
						��ʶ��0x1ff
						������޵�ѹ����-30000~30000
						e��0x205��ѹ����
						f��0x206��ѹ����
						g��0x207��ѹ����
						h��0x208��ѹ����
  * @retval None
  */
void gimbalCurrent_send(int a, int b, int c, int d)
{
    CanTxMsg tx_message3508;
	
    tx_message3508.IDE = CAN_ID_STD;    
    tx_message3508.RTR = CAN_RTR_DATA; 
    tx_message3508.DLC = 0x08;    
    tx_message3508.StdId = 0x200;
	
    a = LIMIT_MAX_MIN(a, Current_Max, -Current_Max);
    b = LIMIT_MAX_MIN(b, Current_Max, -Current_Max);
    c = LIMIT_MAX_MIN(c, Current_Max, -Current_Max);
    d = LIMIT_MAX_MIN(d, Current_Max, -Current_Max);
		
    tx_message3508.Data[0] = (unsigned char)((a >> 8) & 0xff);
    tx_message3508.Data[1] = (unsigned char)(a & 0xff);  
    tx_message3508.Data[2] = (unsigned char)((b >> 8) & 0xff);
    tx_message3508.Data[3] = (unsigned char)(b & 0xff);
    tx_message3508.Data[4] = (unsigned char)((c >> 8) & 0xff);
    tx_message3508.Data[5] = (unsigned char)(c & 0xff);
    tx_message3508.Data[6] = (unsigned char)((d >> 8) & 0xff); 
    tx_message3508.Data[7] = (unsigned char)(d & 0xff);

		CAN_Transmit(CAN1,&tx_message3508);
		
}
/**
  * @brief  ��������ת����
	* @param  Inte:����
  * @retval �����ȸ�����
  */
float Int_To_Float(int Inte)
{
	return Inte;
}
/**
  * @brief  ������ת��������
	* @param  Inte:����
  * @retval ����
  */
int Float_To_Int(float Floa)
{
	return Floa;
}
/**
  * @brief  ����CAN2
  * @param  None
  * @retval None
  */
void CAN2_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	CAN_InitTypeDef        CAN_InitStructure;
	CAN_FilterInitTypeDef  CAN_FilterInitStructure;

	/* ��GPIOʱ�ӡ�AFIOʱ�ӣ�CANʱ�� */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2, ENABLE);

	/* CAN2 RX PB12 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	/* CAN2 TX PB13 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);								

	/* CAN2 Enabling interrupt */								 	  
	NVIC_Config(CAN2_RX1_IRQn, 2, 1);
	NVIC_Config(CAN2_TX_IRQn, 2, 2);
	
	/* CAN  BaudRate = RCC_APB1PeriphClock/(CAN_SJW+CAN_BS1+CAN_BS2)/CAN_Prescaler */
	CAN_DeInit(CAN2);
	CAN_StructInit(&CAN_InitStructure);   

	CAN_InitStructure.CAN_TTCM=DISABLE;
	CAN_InitStructure.CAN_ABOM=DISABLE;
	CAN_InitStructure.CAN_AWUM=DISABLE;
	CAN_InitStructure.CAN_NART=ENABLE;
	CAN_InitStructure.CAN_RFLM=DISABLE;
	CAN_InitStructure.CAN_TXFP=DISABLE;
	CAN_InitStructure.CAN_Mode=CAN_Mode_Normal;   
	//CAN_InitStructure.CAN_Mode=CAN_Mode_LoopBack;
	CAN_InitStructure.CAN_SJW=CAN_SJW_1tq;
	CAN_InitStructure.CAN_BS1=CAN_BS1_5tq;  
	CAN_InitStructure.CAN_BS2=CAN_BS2_3tq;	
	CAN_InitStructure.CAN_Prescaler=4;

	CAN_Init(CAN2,&CAN_InitStructure);   // CAN2													
	
	CAN_FilterInitStructure.CAN_FilterNumber=15;	// 
	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdList;	 // ��ʶ������λģʽ
	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;   // 32λ������
	CAN_FilterInitStructure.CAN_FilterIdHigh=0x100 << 5;			// ��������ʶ��
	CAN_FilterInitStructure.CAN_FilterIdLow=0 | CAN_ID_STD;				
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh= 0x101 << 5;		
	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0 | CAN_ID_STD;
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_FIFO1;	 // FIFO0ָ�������
	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;
	CAN_FilterInit(&CAN_FilterInitStructure);
	
	CAN_ITConfig(CAN2,CAN_IT_FMP1,ENABLE); 
	CAN_ITConfig(CAN2,CAN_IT_TME,ENABLE);
}


/**
  * @brief  CAN2�����жϣ�FIFO1
  * @param  None
  * @retval None
  */
CanRxMsg rx_message_1;	

void CAN2_RX1_IRQHandler(void)
{
	if(CAN_GetITStatus(CAN2, CAN_IT_FMP1) != RESET) 
	{
    CAN_Receive(CAN2, CAN_FIFO1, &rx_message_1);	
		if (rx_message_1.StdId == 0x101)
		{
			memcpy(&yaw_angle,&rx_message_1.Data,4);
			memcpy(&gz,&(rx_message_1.Data[4]),4);
			
		}
		
		Discnect_Flag.Gyro_Cnt = 0;
		Discnect_Flag.Gyro_Connect = 0;
		if(Gyro_Initial_Flag == 0)
		{
			Gyro_Initial_Flag = 1;
		}
		CAN_ClearITPendingBit(CAN2, CAN_IT_FMP1);
		Discnect_Flag.Can2_Cnt=0;
		Discnect_Flag.Can2_Connect=0;
	}
}

/**
  * @brief  CAN2�����ж�
  * @param  None
  * @retval None
  */
void CAN2_TX_IRQHandler(void)
{
	if(CAN_GetITStatus(CAN2, CAN_IT_TME) != RESET) 
	{
		CAN_ClearITPendingBit(CAN2, CAN_IT_TME);
	}
}

