/*---------------------------------------------------------------------------*/
/** 
  **@brief 云台电机pid计算输出
  **
  **/
/*---------------------------------------------------------------------------*/


#include "main.h"
u8 Gimbal_Power_Flag = 1;
int putterpos_init;
int push_pull_pos_init1;
int push_pull_pos_init2;

float PitchSpeedLlimit = 10000;
float YawSpeedLimit = 3000;
float RechargeSpeedLimit=7000;
float PushPullSpeedLimit=5000;
float RealPos[MotorNum];
float LastPos[MotorNum];
int CurrentSend[MotorNum];
float RealSpeed[MotorNum];

PID	pid_gimbal_speed_recharge,pid_gimbal_speed_pushpull1,pid_gimbal_speed_pushpull2;
PID	pid_gimbal_pos_recharge,pid_gimbal_pos_pushpull1,pid_gimbal_pos_pushpull2;

extern rmc620_t Gimbal_Motor[MotorNum];//[0]pitch[1]yaw[3]PushPull

void pid_gimbal_Cal_rc(void)
{
	//位置环
	pid_gimbal_speed_recharge.SetPoint =LIMIT_MAX_MIN(PID_Calc(&pid_gimbal_pos_recharge,RealPos[2]),RechargeSpeedLimit,-RechargeSpeedLimit);
	pid_gimbal_speed_pushpull1.SetPoint=LIMIT_MAX_MIN(PID_Calc(&pid_gimbal_pos_pushpull1,RealPos[0]),PushPullSpeedLimit,-PushPullSpeedLimit);
	pid_gimbal_speed_pushpull2.SetPoint=LIMIT_MAX_MIN(PID_Calc(&pid_gimbal_pos_pushpull2,RealPos[1]),PushPullSpeedLimit,-PushPullSpeedLimit);

	//速度环
	RealSpeed[0] = Int_To_Float(Gimbal_Motor[0].speed);//PushPull1
	CurrentSend[0] = (int)PID_Calc(&pid_gimbal_speed_pushpull1,RealSpeed[0]);
	
	RealSpeed[1] = Int_To_Float(Gimbal_Motor[1].speed);//PushPull2
	CurrentSend[1] = (int)PID_Calc(&pid_gimbal_speed_pushpull2,RealSpeed[1]);
	
	RealSpeed[2] = Int_To_Float(Gimbal_Motor[2].speed);//Recharge
	CurrentSend[2] = (int)PID_Calc(&pid_gimbal_speed_recharge,RealSpeed[2]);
	


	
	//201换弹，202拉线
	//掉电模式发送电流为0；
	if(Gimbal_Power_Flag == 0)
	{
		for(int i=0;i<MotorNum;i++)
		{
			CurrentSend[i]=0;
		}
	}
}
/**
  * @brief  云台电机位置Pid初始化
  * @param  None
  * @retval None
  */ 
void pid_gimbal_Pos_init(void)
{
	push_pull_pos_init1=Gimbal_Motor[0].angle;
	push_pull_pos_init2=Gimbal_Motor[1].angle;
	putterpos_init = Gimbal_Motor[2].angle;
	
	pid_gimbal_pos_recharge.SetPoint = Gimbal_Motor[2].angle;
	pid_gimbal_pos_recharge.P = 0.5f;
	pid_gimbal_pos_recharge.I = 0.015f;
	pid_gimbal_pos_recharge.D = 0.0f;
	pid_gimbal_pos_recharge.IMax = 1000;
	
	pid_gimbal_pos_pushpull1.SetPoint = Gimbal_Motor[0].angle;
	pid_gimbal_pos_pushpull1.P = 0.5f;				//原0.75
	pid_gimbal_pos_pushpull1.I = 0.0f;
	pid_gimbal_pos_pushpull1.D = 0.0f;
	pid_gimbal_pos_pushpull1.IMax = 1000;
	
	pid_gimbal_pos_pushpull2.SetPoint = Gimbal_Motor[1].angle;
	pid_gimbal_pos_pushpull2.P = 0.5f;				//原0.75
	pid_gimbal_pos_pushpull2.I = 0.0f;
	pid_gimbal_pos_pushpull2.D = 0.0f;
	pid_gimbal_pos_pushpull2.IMax = 1000;
}
/**
  * @brief  云台电机速度Pid初始化
  * @param  None
  * @retval None
  */ 
void pid_gimbal_Speed_init(void)
{	
	pid_gimbal_speed_recharge.SetPoint = 0;
	pid_gimbal_speed_recharge.P = 14.0f;			//原11.0 0.0 0.0
	pid_gimbal_speed_recharge.I = 0.05f;
	pid_gimbal_speed_recharge.D = 0.0f;
	pid_gimbal_speed_recharge.IMax = 500;
	
	pid_gimbal_speed_pushpull1.SetPoint=0;
	pid_gimbal_speed_pushpull1.P=14.0f;				//原11.0 0.0 0.0
	pid_gimbal_speed_pushpull1.I=0.075f;
	pid_gimbal_speed_pushpull1.D=0.0f;
	pid_gimbal_speed_pushpull1.IMax=500;
	
	pid_gimbal_speed_pushpull2.SetPoint=0;
	pid_gimbal_speed_pushpull2.P=14.0f;				//原11.0 0.0 0.0
	pid_gimbal_speed_pushpull2.I=0.075f;
	pid_gimbal_speed_pushpull2.D=0.0f;
	pid_gimbal_speed_pushpull2.IMax=500;
}
/**
  * @brief  换弹电机定位移动
  * @param  None
  * @retval None
  */ 
void Recharge_Pos_Move(int position)
{
	pid_gimbal_pos_recharge.SetPoint = putterpos_init + position;
}


/**
	* @brief 拉线电机定位移动
	* @param 转动相对位置
	*	@retval None
	*/
void PushPull_Pos_Move(int position)
{
	pid_gimbal_pos_pushpull1.SetPoint = push_pull_pos_init1 + position;
	pid_gimbal_pos_pushpull2.SetPoint = push_pull_pos_init2 + position;

}
