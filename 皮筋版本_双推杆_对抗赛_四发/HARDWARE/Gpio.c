/*---------------------------------------------------------------------------*/
/** 
  **@brief �ⲿ�ж�����
  **
  **/
/*---------------------------------------------------------------------------*/
#include "main.h"
u32 interrupt_cnt=0;
u32 out_or_back;
u32 last_out_or_back;
u32 triggercnt;//����Ϊ�����أ�ż��Ϊ�½���
u32 usget;
u32 counter=0;
u8 genuine=0;
extern u32 pull_start_time;
extern u32 pull_over_time;
extern u8 launch_times;
extern u8 recharge_enable;
extern u32 syscnt;
extern u8  auto_ctrl_flag;
extern u8 recharge_work_flag ;//��־������ʼ����������������0�����Կ�ʼ��1
extern u8 charge_flag;
extern u8 pull_work_flag;
extern u8 push_work_flag;
extern int putterpos_init;
float us;
float Lspeed;

/**
 **@brief  PB0�г̿��������ж�
 **@retval None
 **/
void gpio_exti_Config()
{	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
	GPIO_InitTypeDef gpio;
	EXTI_InitTypeDef exti;
	gpio.GPIO_Pin=GPIO_Pin_1;
	gpio.GPIO_Mode=GPIO_Mode_Out_PP;
	gpio.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &gpio);	//PB1�������
	
	gpio.GPIO_Pin=GPIO_Pin_0;
	gpio.GPIO_Mode= GPIO_Mode_IPD;
	gpio.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &gpio); //PB0��������
	
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB,GPIO_PinSource0);
	
	exti.EXTI_Line=EXTI_Line0;
	exti.EXTI_Mode=EXTI_Mode_Interrupt;
	exti.EXTI_Trigger=EXTI_Trigger_Rising;//�����ش���
	exti.EXTI_LineCmd=ENABLE;
	EXTI_Init(&exti);
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1); 
		
	NVIC_InitTypeDef NVIC_InitStructure;//����ṹ��
	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn; //ʹ���ⲿ�ж����ڵ�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01; //��ռ���ȼ� 1�� 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00; //�����ȼ� 1
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //ʹ���ⲿ�ж�ͨ�� 

	NVIC_Init(&NVIC_InitStructure); //���ݽṹ����Ϣ�������ȼ���ʼ��
		
	GPIO_SetBits(GPIOB,GPIO_Pin_1);		//����ߵ�ƽ
		
}


/**
 **@brief  yaw���Ƹ˵����������
 **@retval None
 **/
	
void	yaw_gpio_Config(void)
{
	GPIO_InitTypeDef gpio;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);	
	gpio.GPIO_Pin=GPIO_Pin_8;
	gpio.GPIO_Mode=GPIO_Mode_Out_PP;
	gpio.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOC,&gpio);
	
	gpio.GPIO_Pin=GPIO_Pin_10;
	gpio.GPIO_Mode=GPIO_Mode_Out_PP;
	gpio.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&gpio);
	
}

/**
  * @brief  ����1����,�����ⲿ�жϲ���
  * @param  None
  * @retval None 	
  */ 
void GPIO_Photocell(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure); 
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure); 
	
	GPIO_SetBits(GPIOA,GPIO_Pin_2);
	//�����ⲿ�ж�
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
		
		GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource3);
		
		EXTI_InitTypeDef EXTI_InitStructure;//�����ʼ���ṹ��
		
		EXTI_InitStructure.EXTI_Line=EXTI_Line3; 
		EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
		EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;//EXTI_Trigger_Rising_Falling;
		EXTI_InitStructure.EXTI_LineCmd = ENABLE;

		EXTI_Init(&EXTI_InitStructure);//���ݽṹ����Ϣ���г�ʼ��
	}
	//�ж����ȼ�
	{
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1); 
		
		NVIC_InitTypeDef NVIC_InitStructure;//����ṹ��
		NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn; //ʹ���ⲿ�ж����ڵ�ͨ��
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01; //��ռ���ȼ� 0�� 
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00; //�����ȼ� 0
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //ʹ���ⲿ�ж�ͨ�� 

		NVIC_Init(&NVIC_InitStructure); //���ݽṹ����Ϣ�������ȼ���ʼ��
	}
}

/**
  * @brief  ��ȡ����1�ź�
  * @param  None
  * @retval 1Ϊ�ڵ���0Ϊ���ڵ�
  */ 
u8 Get_Photocell_1(void)
{
	return GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_3);
}
/**
  * @brief  ��ȡ����2�ź�
  * @param  None
  * @retval 1Ϊ�ڵ���0Ϊ���ڵ�
  */ 
//u8 Get_Photocell_2(void)
//{
//	return GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_11);
//}
/**
  * @brief  ����1�ⲿ�жϲ���		
  * @param  None
  * @retval None
  */ 
void EXTI3_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line3)!=RESET)//�ж�ĳ�����ϵ��ж��Ƿ��� 
	{
 		EXTI_ClearITPendingBit(EXTI_Line3);   //��� LINE �ϵ��жϱ�־λ
		
			us10_on_off=1;										//TIM2��ʱ����
			triggercnt ++;
			if(triggercnt % 2 == 0)//2��������
			{
				usget = Get_us10();//�������μ�ʱ��ʱ������΢�룩
				us = (float)usget;
				Lspeed = 1000 * 194.0f / us;						//���������ž��룬��Ҫ����
				us10_reset();
			}
	}     
}

/**
 **@brief  �г̿��ش����ж�
 **@retval None
 **/

void EXTI0_IRQHandler(void)		//�������г̿���
{
	if(EXTI_GetITStatus(EXTI_Line0)!=RESET)//ȷʵ�ܽ����ж�
	{
		if(auto_ctrl_flag)
		{
			while(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_0))
			{
				counter++;
				if(counter>100)
				{
					genuine=1;
					counter=0;
					GPIO_ResetBits(GPIOB,GPIO_Pin_1);//�����͵�ƽ��������´η���ʱ������
				}		
			}
		if(genuine)
		{
			interrupt_cnt++;
			if(auto_ctrl_flag)		//PullWork���AutoCtrl_Flag��1
			{
				pull_work_flag=0;//ֹͣ���߱�־λ
				push_work_flag=1;//���߱�־λ��Ч
//				if(pull_over_time-pull_start_time>1000)			//���ȷʵ�����ȥ��
//				{
					if((!charge_flag)&&(launch_times==1))	//���ǿ�ʼ����װ�����ǵ�һ�η���
					{
						recharge_work_flag=1;//���Կ�ʼ����
					}
//				}
			}
		}
		counter=0;
		genuine=0;
		
		}
}
	EXTI_ClearITPendingBit(EXTI_Line0);
}


/**
	* @brief  ���ܼ�⻬�鷢���򵹻�
		out_or_back���Ƿ��������źţ�����/����һ�������ڵ����Σ�out_or_back+1
	* @param  Num:������1/2
	* @retval 1:������0������,8:����	2020.11.5
   */ 

u32 Photocell_Detect(void)
{

	u8 Return;
	if(out_or_back==0)					//��ʼ���г���
	{
		Return=0;
	}
	else if(last_out_or_back==out_or_back)				//û�����������ж�
	{
		Return=0;
	}
	else
	{
				if(out_or_back % 2 != 0)
					Return=1;
				else
					Return=0;
	}
	last_out_or_back=out_or_back;
	return Return;
	
}
