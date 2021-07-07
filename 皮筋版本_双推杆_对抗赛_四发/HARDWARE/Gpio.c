/*---------------------------------------------------------------------------*/
/** 
  **@brief 外部中断配置
  **
  **/
/*---------------------------------------------------------------------------*/
#include "main.h"
u32 interrupt_cnt=0;
u32 out_or_back;
u32 last_out_or_back;
u32 triggercnt;//奇数为上升沿，偶数为下降沿
u32 usget;
u32 counter=0;
u8 genuine=0;
extern u32 pull_start_time;
extern u32 pull_over_time;
extern u8 launch_times;
extern u8 recharge_enable;
extern u32 syscnt;
extern u8  auto_ctrl_flag;
extern u8 recharge_work_flag ;//标志换弹开始结束，换弹结束是0，可以开始是1
extern u8 charge_flag;
extern u8 pull_work_flag;
extern u8 push_work_flag;
extern int putterpos_init;
float us;
float Lspeed;

/**
 **@brief  PB0行程开关配置中断
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
	GPIO_Init(GPIOB, &gpio);	//PB1推挽输出
	
	gpio.GPIO_Pin=GPIO_Pin_0;
	gpio.GPIO_Mode= GPIO_Mode_IPD;
	gpio.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &gpio); //PB0浮空输入
	
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB,GPIO_PinSource0);
	
	exti.EXTI_Line=EXTI_Line0;
	exti.EXTI_Mode=EXTI_Mode_Interrupt;
	exti.EXTI_Trigger=EXTI_Trigger_Rising;//上升沿触发
	exti.EXTI_LineCmd=ENABLE;
	EXTI_Init(&exti);
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1); 
		
	NVIC_InitTypeDef NVIC_InitStructure;//定义结构体
	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn; //使能外部中断所在的通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01; //抢占优先级 1， 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00; //子优先级 1
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //使能外部中断通道 

	NVIC_Init(&NVIC_InitStructure); //根据结构体信息进行优先级初始化
		
	GPIO_SetBits(GPIOB,GPIO_Pin_1);		//输出高电平
		
}


/**
 **@brief  yaw轴推杆电机引脚配置
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
  * @brief  光电管1配置,包含外部中断测速
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
	//配置外部中断
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
		
		GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource3);
		
		EXTI_InitTypeDef EXTI_InitStructure;//定义初始化结构体
		
		EXTI_InitStructure.EXTI_Line=EXTI_Line3; 
		EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
		EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;//EXTI_Trigger_Rising_Falling;
		EXTI_InitStructure.EXTI_LineCmd = ENABLE;

		EXTI_Init(&EXTI_InitStructure);//根据结构体信息进行初始化
	}
	//中断优先级
	{
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1); 
		
		NVIC_InitTypeDef NVIC_InitStructure;//定义结构体
		NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn; //使能外部中断所在的通道
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01; //抢占优先级 0， 
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00; //子优先级 0
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //使能外部中断通道 

		NVIC_Init(&NVIC_InitStructure); //根据结构体信息进行优先级初始化
	}
}

/**
  * @brief  获取光电管1信号
  * @param  None
  * @retval 1为遮挡，0为无遮挡
  */ 
u8 Get_Photocell_1(void)
{
	return GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_3);
}
/**
  * @brief  获取光电管2信号
  * @param  None
  * @retval 1为遮挡，0为无遮挡
  */ 
//u8 Get_Photocell_2(void)
//{
//	return GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_11);
//}
/**
  * @brief  光电管1外部中断测速		
  * @param  None
  * @retval None
  */ 
void EXTI3_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line3)!=RESET)//判断某个线上的中断是否发生 
	{
 		EXTI_ClearITPendingBit(EXTI_Line3);   //清除 LINE 上的中断标志位
		
			us10_on_off=1;										//TIM2计时开启
			triggercnt ++;
			if(triggercnt % 2 == 0)//2的整数倍
			{
				usget = Get_us10();//返回两次计时的时间间隔（微秒）
				us = (float)usget;
				Lspeed = 1000 * 194.0f / us;						//托盘两个脚距离，需要测量
				us10_reset();
			}
	}     
}

/**
 **@brief  行程开关触发中断
 **@retval None
 **/

void EXTI0_IRQHandler(void)		//触发了行程开关
{
	if(EXTI_GetITStatus(EXTI_Line0)!=RESET)//确实能进入中断
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
					GPIO_ResetBits(GPIOB,GPIO_Pin_1);//拉至低电平，当电机下次发射时再拉高
				}		
			}
		if(genuine)
		{
			interrupt_cnt++;
			if(auto_ctrl_flag)		//PullWork会把AutoCtrl_Flag置1
			{
				pull_work_flag=0;//停止拉线标志位
				push_work_flag=1;//推线标志位有效
//				if(pull_over_time-pull_start_time>1000)			//舵机确实发射出去了
//				{
					if((!charge_flag)&&(launch_times==1))	//不是开始比赛装弹且是第一次发射
					{
						recharge_work_flag=1;//可以开始换弹
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
	* @brief  光电管检测滑块发出或倒回
		out_or_back就是发出回来信号，发出/回来一个过程遮挡两次，out_or_back+1
	* @param  Num:滑块编号1/2
	* @retval 1:发出，0：倒回,8:错误	2020.11.5
   */ 

u32 Photocell_Detect(void)
{

	u8 Return;
	if(out_or_back==0)					//初始运行程序
	{
		Return=0;
	}
	else if(last_out_or_back==out_or_back)				//没连续进光电管中断
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
