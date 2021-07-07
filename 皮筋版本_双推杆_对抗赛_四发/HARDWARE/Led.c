/*---------------------------------------------------------------------------*/
/** 
  **@brief ����LED������
  **
  **/
/*---------------------------------------------------------------------------*/
  
  #include "main.h"

void LED_Config(void)
{
	GPIO_InitTypeDef gpio;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

	gpio.GPIO_Pin = GPIO_Pin_13|GPIO_Pin_14;
	gpio.GPIO_Mode = GPIO_Mode_Out_PP;
	gpio.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC,&gpio);

	GPIO_ResetBits(GPIOC,GPIO_Pin_13 | GPIO_Pin_14);   //���ǿ���led��
}
/**
  * @brief  LED����˸
	* @param  GPIOx 
	*					GPIO_Pin
	*					Period :	��˸����/ms
	*					On : ��ʱ��/ms
  * @retval һ����˸��������������
  */
void Led_Twinkle1(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin,uint16_t Period,uint16_t On)
{
	u8 syscnt;
	syscnt = GetSyscnt();
	if(syscnt % Period < On)
	{
		GPIO_SetBits(GPIOx,GPIO_Pin);
	}
	else
	{
		GPIO_ResetBits(GPIOx,GPIO_Pin);
	}
}
/**
  * @brief  LED����˸
	* @param  GPIOx 
	*					GPIO_Pin
	*					Period :	��˸����/ms
	*					On : ��ʱ��/ms
  * @retval һ����˸�������������
  */
void Led_Twinkle2(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin,uint16_t Period,uint16_t On)
{
	u8 syscnt;
	syscnt = GetSyscnt();
	if(syscnt % Period < On)
	{
		GPIO_SetBits(GPIOx,GPIO_Pin);
	}
	else
	{
		GPIO_ResetBits(GPIOx,GPIO_Pin);
	}
}
/**
  * @brief  ˫LED��Ƶ��˸
	* @param  None
  * @retval None
  */
void LedStatus0(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin,GPIO_TypeDef* GPIOx_, uint16_t GPIO_Pin_)
{
	Led_Twinkle1(GPIOx,GPIO_Pin,2000,1000);
	Led_Twinkle1(GPIOx_,GPIO_Pin_,500,250);
}
/**
  * @brief  ˫LEDͬƵͬʱ��˸
	* @param  None
  * @retval None
  */
void LedStatus1(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin,GPIO_TypeDef* GPIOx_, uint16_t GPIO_Pin_)
{
	Led_Twinkle1(GPIOx,GPIO_Pin,500,250);
	Led_Twinkle1(GPIOx_,GPIO_Pin_,500,250);
}
/**
  * @brief  ˫LEDͬƵ������˸
	* @param  None
  * @retval None
  */
void LedStatus2(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin,GPIO_TypeDef* GPIOx_, uint16_t GPIO_Pin_)
{
	Led_Twinkle1(GPIOx,GPIO_Pin,500,250);
	Led_Twinkle2(GPIOx_,GPIO_Pin_,500,250);
}
/**
  * @brief  LED1��˸��LED2��
	* @param  None
  * @retval None
  */
void LedStatus3(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin,GPIO_TypeDef* GPIOx_, uint16_t GPIO_Pin_)
{
	Led_Twinkle1(GPIOx,GPIO_Pin,500,250);
	Led_Twinkle2(GPIOx_,GPIO_Pin_,500,500);
}
/**
  * @brief  LED2��˸��LED1��
	* @param  None
  * @retval None
  */
void LedStatus4(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin,GPIO_TypeDef* GPIOx_, uint16_t GPIO_Pin_)
{
	Led_Twinkle1(GPIOx,GPIO_Pin,500,250);
	Led_Twinkle2(GPIOx_,GPIO_Pin_,500,500);
}
/**
  * @brief  һ����
	* @param  None
  * @retval None
  */
void LedStatus5(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin,GPIO_TypeDef* GPIOx_, uint16_t GPIO_Pin_)
{
	Led_Twinkle2(GPIOx_,GPIO_Pin_,500,500);
	Led_Twinkle2(GPIOx,GPIO_Pin,500,500);
}
/**
  * @brief  һ����
	* @param  None
  * @retval None
  */
void LedStatus6(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin,GPIO_TypeDef* GPIOx_, uint16_t GPIO_Pin_)
{
	Led_Twinkle1(GPIOx_,GPIO_Pin_,500,500);
	Led_Twinkle1(GPIOx,GPIO_Pin,500,500);
}
/**
  * @brief  1��2��
	* @param  None
  * @retval None
  */
void LedStatus7(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin,GPIO_TypeDef* GPIOx_, uint16_t GPIO_Pin_)
{
	Led_Twinkle1(GPIOx,GPIO_Pin,500,500);
	Led_Twinkle2(GPIOx_,GPIO_Pin_,500,500);
}
/**
  * @brief  2��1��
	* @param  None
  * @retval None
  */
void LedStatus8(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin,GPIO_TypeDef* GPIOx_, uint16_t GPIO_Pin_)
{
	Led_Twinkle1(GPIOx_,GPIO_Pin_,500,500);
	Led_Twinkle2(GPIOx,GPIO_Pin,500,500);
}
/**
  * @brief  LED1��˸��LED2����
	* @param  None
  * @retval None
  */
void LedStatus9(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin,GPIO_TypeDef* GPIOx_, uint16_t GPIO_Pin_)
{
	Led_Twinkle1(GPIOx,GPIO_Pin,500,250);
	Led_Twinkle1(GPIOx_,GPIO_Pin_,500,500);
}
/**
  * @brief  LED2��˸��LED1����
	* @param  None
  * @retval None
  */
void LedStatus10(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin,GPIO_TypeDef* GPIOx_, uint16_t GPIO_Pin_)
{
	Led_Twinkle1(GPIOx_,GPIO_Pin_,500,250);
	Led_Twinkle1(GPIOx,GPIO_Pin,500,500);
}
/**
  * @brief  ˫LEDͬƵͬʱ��˸������
	* @param  None
  * @retval None
  */
void LedStatus11(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin,GPIO_TypeDef* GPIOx_, uint16_t GPIO_Pin_)
{
	Led_Twinkle1(GPIOx,GPIO_Pin,2000,1000);
	Led_Twinkle1(GPIOx_,GPIO_Pin_,2000,1000);
}
/**
  * @brief  ˫LEDͬƵ������˸������
	* @param  None
  * @retval None
  */
void LedStatus12(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin,GPIO_TypeDef* GPIOx_, uint16_t GPIO_Pin_)
{
	Led_Twinkle1(GPIOx,GPIO_Pin,2000,1000);
	Led_Twinkle2(GPIOx_,GPIO_Pin_,2000,1000);
}





void Disconnect_Report(u32 Disconnect_Flag)
{
	if(Get_u32_nth(Disconnect_Flag,0) == 0x0001)
	{
		LedStatus1(GPIOC,GPIO_Pin_13,GPIOC,GPIO_Pin_14);
	}
	else if(Get_u32_nth(Disconnect_Flag,1) == 0x0001)
	{
		LedStatus2(GPIOC,GPIO_Pin_13,GPIOC,GPIO_Pin_14);
	}
	else if(Get_u32_nth(Disconnect_Flag,2) == 0x0001)
	{
		LedStatus3(GPIOC,GPIO_Pin_13,GPIOC,GPIO_Pin_14);
	}
	else if(Get_u32_nth(Disconnect_Flag,3) == 0x0001)
	{
		LedStatus4(GPIOC,GPIO_Pin_13,GPIOC,GPIO_Pin_14);
	}
	else if(Get_u32_nth(Disconnect_Flag,4) == 0x0001)
	{
		LedStatus5(GPIOC,GPIO_Pin_13,GPIOC,GPIO_Pin_14);
	}
	else if(Get_u32_nth(Disconnect_Flag,5) == 0x0001)
	{
		LedStatus6(GPIOC,GPIO_Pin_13,GPIOC,GPIO_Pin_14);
	}
	else if(Get_u32_nth(Disconnect_Flag,6) == 0x0001)
	{
		LedStatus7(GPIOC,GPIO_Pin_13,GPIOC,GPIO_Pin_14);
	}
	else if(Get_u32_nth(Disconnect_Flag,7) == 0x0001)
	{
		LedStatus8(GPIOC,GPIO_Pin_13,GPIOC,GPIO_Pin_14);
	}
	else if(Get_u32_nth(Disconnect_Flag,8) == 0x0001)
	{
		LedStatus9(GPIOC,GPIO_Pin_13,GPIOC,GPIO_Pin_14);
	}
	else if(Get_u32_nth(Disconnect_Flag,9) == 0x0001)
 {
		LedStatus10(GPIOC,GPIO_Pin_13,GPIOC,GPIO_Pin_14);
	}
	else if(Get_u32_nth(Disconnect_Flag,10) == 0x0001)
	{
		LedStatus11(GPIOC,GPIO_Pin_13,GPIOC,GPIO_Pin_14);
	}
	else if(Get_u32_nth(Disconnect_Flag,11) == 0x0001)
	{
		LedStatus12(GPIOC,GPIO_Pin_13,GPIOC,GPIO_Pin_14);
	}
	else
	{
		LedStatus0(GPIOC,GPIO_Pin_13,GPIOC,GPIO_Pin_14);
	}
}
