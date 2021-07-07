/** @˵��	˫�Ƹ�Ƥ���ܴ���
 ** @����	���
 ** @����	2021.6
 ** @�汾   V1.0
 **/


#include "main.h"
/*
�������ã�
																																									
PA0,1:�ͷŶ��(����)
PB6,7:�����Ƹ˵��	
PB0,1���г̿���
PC6,7:pitch�Ƹ˵��																												
//PA2������1���չ�3.3V����	Tx																										
//PA3������1���չ��ź�ͨ��	Rx																						
//P22 CAN2_H CAN2_L ���ܷ��͹ܹ���
PC8��PB10	yaw�Ƹ˵��																																																																																																																																																��yaw���Ƹ˵��			
��������CAN����		
�������0,1,2 �ֱ��Ӧ PushPull1��PushPull2��Recharge
*/			
u32 syscnt;
u32 disconnect_flag_all;//���߹۲��־λ
Disconnect_Flag Discnect_Flag;//���߽ṹ��
Status_ Status;
RCC_ClocksTypeDef rcc;
u8 auto_ctrl_flag;
extern int CurrentSend[MotorNum];
extern u32 triggercnt,us10cnt;//����Ϊ�����أ�ż��Ϊ�½���
extern u8 recharge_work_flag;
extern JudgeReceive_t judge_receive;

int main()
{
	RCC_GetClocksFreq(&rcc);
/*--------------------------��������-----------------------------*/
	LED_Config();
	MG995_TIM5_Config();//���ض����ʼ��
	Pitch_TIM3_Config();//pitch�Ƹ˵����ʼ�� 
	yaw_gpio_Config();//yaw�������ų�ʼ��
	motor_putter_Config();//�Ƹ˳�ʼ��		
	TIM2_Config();//��Ϲ���10us��ʱ
	TIM7_Config();//���뼶��ʱ����
/*------------------------------ͨ��------------------------------*/	
	uart1_Configuration();//����ϵͳͨ��
	usart_Config();//���ڳ�ʼ����ң����
	can1_Config();//��̨3508��6020
	delay_ms(1000);
/*---------------------------������ʼ��---------------------------*/
	zeroCheck_init();
	pid_gimbal_Pos_init();//���λ�û���ʼ��
	pid_gimbal_Speed_init();//����ٶȻ���ʼ��
	rc_Reset();
	flag_Initialize();
	SysTick_Configuration();	
/*-----------------------------�ⲿ�ж�����----------------------------------*/
	GPIO_Photocell();//��������
	delay_ms(1000);
	{
		triggercnt = 0;
		us10_on_off = 0;
		us10cnt = 0;		
	}
	gpio_exti_Config();//�г̿��������߳�ʼ�������������Ӳ��������жϣ�����ٳ�ʼ��

	while(1)
	{
		
	}	
}
		


void SysTick_Handler(void)
{
/*----------------------------------������----------------------------------*/
	syscnt ++;
/*-----------------------------ϵͳ�������-----------------------------------*/
	disconnect_flag_all = offlineDetect(&Discnect_Flag);//��ȡ������Ϣ,����ͨ���۲���������жϣ�
	Disconnect_Report(disconnect_flag_all);//led��ӳ����״̬
	zeroCheck_cal();//�����λ�û������⣬���� Motor_Angle[i]
	zeroCheck_execution();//��������������λ�ã����Ϊ RealPos[i]
	remoteConvert(rc_ctrl);//ͨ��ң�ؿ��Ƽ���/ң��/����״̬�л�����
	statusAct(Status);//����ң����״̬�������ã����������ź�
	judgementAct(judge_receive);//���ݲ���ϵͳ�ṹ����������ź�
	judge_pc_Control();//�����źŲ��������
/*----------------------------������������----------------------------------*/	
	pid_gimbal_Cal_rc();//���ߵ�����������pid����
/*-------------------------���������������������߻���3508---------------------*/
	gimbalCurrent_send(CurrentSend[0],CurrentSend[1],CurrentSend[2],0);
/*--------------------------�鿴�Ƿ���뵹�ػ�������---------------------------*/
	pullworkTask();
	rechargeTask();
}
/**
  * @brief  Systick��ʼ��
  * @param  None
  * @retval None
  */
void SysTick_Configuration(void)
{
  /* Setup SysTick Timer for 5 msec interrupts  */
  if (SysTick_Config(72000000/ 1000))                //SysTick���ú��� T = 1 / 1000 = 1ms
  { 
    /* Capture error */ 
    while (1);
  }  
 /* Configure the SysTick handler priority */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1); 
	NVIC_SetPriority(SysTick_IRQn, 0x00);                       //SysTick�ж����ȼ�����
}
/**
  * @brief  ��ȡu32�ĵ�nλ(���߱�־��)
  * @param  num32��u32����
	*					n:��nλ
  * @retval u32�ĵ�nλ
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
  * @brief  ��ȡ����ʱ��
  * @param  None
  * @retval ϵͳ����ʱ��ms
  */
u32 GetSyscnt(void)
{
	return syscnt;
}
