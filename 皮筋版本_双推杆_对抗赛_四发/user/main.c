/** @说明	双推杆皮筋发射架代码
 ** @作者	贾宇凡
 ** @日期	2021.6
 ** @版本   V1.0
 **/


#include "main.h"
/*
引脚配置：
																																									
PA0,1:释放舵机(两个)
PB6,7:换弹推杆电机	
PB0,1：行程开关
PC6,7:pitch推杆电机																												
//PA2：光电管1接收管3.3V供电	Tx																										
//PA3：光电管1接收管信号通道	Rx																						
//P22 CAN2_H CAN2_L 光电管发送管供电
PC8、PB10	yaw推杆电机																																																																																																																																																：yaw轴推杆电机			
其余电机接CAN总线		
电机数组0,1,2 分别对应 PushPull1、PushPull2、Recharge
*/			
u32 syscnt;
u32 disconnect_flag_all;//掉线观察标志位
Disconnect_Flag Discnect_Flag;//断线结构体
Status_ Status;
RCC_ClocksTypeDef rcc;
u8 auto_ctrl_flag;
extern int CurrentSend[MotorNum];
extern u32 triggercnt,us10cnt;//奇数为上升沿，偶数为下降沿
extern u8 recharge_work_flag;
extern JudgeReceive_t judge_receive;

int main()
{
	RCC_GetClocksFreq(&rcc);
/*--------------------------基础外设-----------------------------*/
	LED_Config();
	MG995_TIM5_Config();//拦截舵机初始化
	Pitch_TIM3_Config();//pitch推杆电机初始化 
	yaw_gpio_Config();//yaw控制引脚初始化
	motor_putter_Config();//推杆初始化		
	TIM2_Config();//配合光电管10us计时
	TIM7_Config();//毫秒级延时配置
/*------------------------------通信------------------------------*/	
	uart1_Configuration();//裁判系统通信
	usart_Config();//串口初始化，遥控器
	can1_Config();//云台3508，6020
	delay_ms(1000);
/*---------------------------参数初始化---------------------------*/
	zeroCheck_init();
	pid_gimbal_Pos_init();//电机位置环初始化
	pid_gimbal_Speed_init();//电机速度环初始化
	rc_Reset();
	flag_Initialize();
	SysTick_Configuration();	
/*-----------------------------外部中断配置----------------------------------*/
	GPIO_Photocell();//光电管配置
	delay_ms(1000);
	{
		triggercnt = 0;
		us10_on_off = 0;
		us10cnt = 0;		
	}
	gpio_exti_Config();//行程开关连接线初始化，容易由于杂波持续进中断，最后再初始化

	while(1)
	{
		
	}	
}
		


void SysTick_Handler(void)
{
/*----------------------------------计数器----------------------------------*/
	syscnt ++;
/*-----------------------------系统检测任务-----------------------------------*/
	disconnect_flag_all = offlineDetect(&Discnect_Flag);//读取断线信息,可以通过观察这个变量判断？
	Disconnect_Report(disconnect_flag_all);//led反映掉线状态
	zeroCheck_cal();//各电机位置环过零检测，变量 Motor_Angle[i]
	zeroCheck_execution();//过零检测输出各电机位置，输出为 RealPos[i]
	remoteConvert(rc_ctrl);//通过遥控控制键盘/遥控/掉电状态切换函数
	statusAct(Status);//根据遥控器状态重新作用，产生控制信号
	judgementAct(judge_receive);//根据裁判系统结构体产生控制信号
	judge_pc_Control();//控制信号操作发射架
/*----------------------------各电机计算输出----------------------------------*/	
	pid_gimbal_Cal_rc();//拉线电机、换弹电机pid控速
/*-------------------------三个电流给定，控制拉线换弹3508---------------------*/
	gimbalCurrent_send(CurrentSend[0],CurrentSend[1],CurrentSend[2],0);
/*--------------------------查看是否进入倒回换弹任务---------------------------*/
	pullworkTask();
	rechargeTask();
}
/**
  * @brief  Systick初始化
  * @param  None
  * @retval None
  */
void SysTick_Configuration(void)
{
  /* Setup SysTick Timer for 5 msec interrupts  */
  if (SysTick_Config(72000000/ 1000))                //SysTick配置函数 T = 1 / 1000 = 1ms
  { 
    /* Capture error */ 
    while (1);
  }  
 /* Configure the SysTick handler priority */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1); 
	NVIC_SetPriority(SysTick_IRQn, 0x00);                       //SysTick中断优先级设置
}
/**
  * @brief  获取u32的第n位(掉线标志用)
  * @param  num32：u32对象
	*					n:第n位
  * @retval u32的第n位
  */
u32 Get_u32_nth(u32 num32,u8 n)
{
	if(n < 32)
	{
		return ((num32 >> n) & 0x0001);
	}
	else 
	{
		return 0;
	}
}
/**
  * @brief  获取运行时间
  * @param  None
  * @retval 系统运行时间ms
  */
u32 GetSyscnt(void)
{
	return syscnt;
}
