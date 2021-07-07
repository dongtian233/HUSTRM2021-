/*---------------------------------------------------------------------------*/
/** 
  **@brief ���Թ�TIM��������
  **
  **/
/*---------------------------------------------------------------------------*/

#include "main.h"

u8 us10_on_off;//1��ʼ��ʱ��0ֹͣ��ʱ
u32 us10cnt;
float RSpeed,Lastrspeed,Linear_Velocity;
float Encoder_Speed;

/**
  * @brief  Tim5���ö�ȡ������
  * @param  None
  * @retval None
  */ 
void TIM5_Cap_Init(void)	
{	 
}
/**
  * @brief  ��ȡ������Ȧ��
  * @param  None
  * @retval None
  */ 
void Get_Encode_Speed(void)
{
}
/**
  * @brief  Tim2����10us��ʱ
  * @param  None
  * @retval None
  */ 
void TIM2_Config(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE); 
	
  TIM_TimeBaseInitStructure.TIM_Period = 720-1;//10us
	TIM_TimeBaseInitStructure.TIM_Prescaler=1-1;  
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseInitStructure);
	TIM_Cmd(TIM2,ENABLE);
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE); 
	 
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1); 
	
	NVIC_InitStructure.NVIC_IRQChannel=TIM2_IRQn; 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x01; 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x01; 
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}
/**
  * @brief  ��Ϲ���1��ʱ
  * @param  None
  * @retval None
  */ 
void TIM2_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM2,TIM_IT_Update)==SET)
	{
		TIM_ClearITPendingBit(TIM2,TIM_IT_Update);
		if(us10_on_off)	us10cnt++;
	}
}
/**
  * @brief  ��ȡus��ʱ
  * @param  None
  * @retval us
  */ 
u32 Get_us10(void)
{
	return us10cnt * 10;
}
/**
  * @brief  us��ʱ����
  * @param  None
  * @retval None
  */ 
void us10_reset(void)
{
	us10cnt = 0;
	us10_on_off=0;
}
