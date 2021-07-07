/*---------------------------------------------------------------------------*/
/** 
  **@brief 系统识别、控制
  **
  **/
/*---------------------------------------------------------------------------*/
#include "main.h"
u8 launch_times=0;
u8 first_enter=1;
u8 first_close_gate=1;
u8 first_emission=0;
u8 timer_enable=0;
u8 recharge_wait=0;
u8 recharge_enable=1;
u16 vcnt;
u16 rcnt;
u16 mousecnt;
u32 pull_start_time=0;
u32 pull_over_time=0;
u32 cntyaw_left=0;
u32 cntpitch_left=0;
u32 cntyaw_right=0;
u32 cntpitch_right=0;
u32 launchcnt=0;
u8 test_back_flag=0;
u8 left_flag=0;
u8 right_flag=0;
u8 charge_flag=0;
u8 launch_flag=0;
u8 launch_flag_last=0;
int pitch_pulse_set;//0~14400对应占空比
u8 dart_attack_target_last=0;
u8 dart_launch_opening_status_last;
u16 operate_launch_cmd_time_last=0;
extern u32 syscnt;
extern u8 recharge_work_flag;
extern u8 pull_work_flag;
extern u8 auto_ctrl_flag;
extern float RealPos[MotorNum];
extern PID pid_speed;
extern PID	pid_gimbal_speed_recharge,pid_gimbal_speed_pushpull1,pid_gimbal_speed_pushpull2;
extern PID	pid_gimbal_pos_recharge,pid_gimbal_pos_pushpull1,pid_gimbal_pos_pushpull2;
extern u32 triggercnt;
extern int putterpos_init;
extern int push_pull_pos_init1,push_pull_pos_init2;
extern int out_or_back;
extern JudgeReceive_t judge_receive;
extern rc_ctrl_t rc_ctrl, rc_ctrl_last;


/**	
 **@brief 根据裁判系统信号生成控制信号
 **@date 2021.6
 **/
void judgementAct(JudgeReceive_t judge)
{
	if(judge.GameStage==4)											//比赛进行中
	{
	if(judge.dart_launch_opening_status==0)							//闸门刚刚开启
	{
		if(first_emission)
		{
			launch_flag=1;		
			first_emission=0;			
		}
	}
//闸门关闭状态
	if(judge.dart_launch_opening_status==1)
	{
		first_emission=1;						//重新使能发射
		if(first_close_gate)					//比赛中第一次关闸门
																					
		{
			charge_flag=1;						//上弹
//			left_flag=1;						//自动瞄准就是前哨站
			first_close_gate=0;
		}
	}
	//原先前哨站，目标变基地
//	if(dart_attack_target_last==0&&judge.dart_attack_target==1)
//	{
//		right_flag=1;
//	}

	dart_attack_target_last=judge.dart_attack_target;
	dart_launch_opening_status_last=judge.dart_launch_opening_status;
	operate_launch_cmd_time_last=judge.operate_launch_cmd_time;//有操作手延迟，不使用
	}

	
}
/**
 **@brief 根据各flag信号实施控制
 **@retval None
 **/
void judge_pc_Control(void)
{
	if(!auto_ctrl_flag)
	{//默认瞄准前哨站，因此不必旋转
			if(left_flag)		//瞄准前哨站
			{
				
				if(cntyaw_left<Yaw_Left_Time)
				{
					GPIO_SetBits(GPIOC,GPIO_Pin_8);
					GPIO_ResetBits(GPIOB,GPIO_Pin_10);
					cntyaw_left++;
				}
				else if(cntyaw_left==Yaw_Left_Time)
				{
					GPIO_ResetBits(GPIOC,GPIO_Pin_8);
					GPIO_ResetBits(GPIOB,GPIO_Pin_10);	//yaw停止
				}
				else
				{
					GPIO_ResetBits(GPIOC,GPIO_Pin_8);
					GPIO_ResetBits(GPIOB,GPIO_Pin_10);	//yaw停止
				}
				
				if(cntpitch_left<Pitch_Down_Time)
				{
					TIM_SetCompare1(TIM3,14399);//调节占空比，改变pitch位置
					TIM_SetCompare2(TIM3,0);
					cntpitch_left++;
				}
				else
				{
					TIM_SetCompare1(TIM3,0);
					TIM_SetCompare2(TIM3,0);					//pitch停止
				}
				if((cntpitch_left==Pitch_Down_Time)&&(cntyaw_left==Yaw_Left_Time))
				{
					left_flag=0;
					
					GPIO_ResetBits(GPIOC,GPIO_Pin_8);
					GPIO_ResetBits(GPIOB,GPIO_Pin_10);	//yaw停止
					
					TIM_SetCompare1(TIM3,0);
					TIM_SetCompare2(TIM3,0);					//pitch停止
				}
			}
			
			if(right_flag)		//瞄准基地
			{
				if(cntyaw_right<(Yaw_Right_Time-Yaw_Left_Time))
				{
					GPIO_SetBits(GPIOC,GPIO_Pin_8);
					GPIO_ResetBits(GPIOB,GPIO_Pin_10);
					cntyaw_right++;
				}
				else
				{
					GPIO_ResetBits(GPIOC,GPIO_Pin_8);
					GPIO_ResetBits(GPIOB,GPIO_Pin_10);	//yaw停止
				}
				
				if(cntpitch_right<(Pitch_Up_Time-Pitch_Down_Time))
				{
					TIM_SetCompare1(TIM3,0);
					TIM_SetCompare2(TIM3,14399);
					cntpitch_right++;
				}
				else
				{
					TIM_SetCompare1(TIM3,0);
					TIM_SetCompare2(TIM3,0);					//pitch停止
				}
				if(cntpitch_right==(Pitch_Up_Time-Pitch_Down_Time)&&cntyaw_right==(Yaw_Right_Time-Yaw_Left_Time))
				{
					right_flag =0;
					
					GPIO_ResetBits(GPIOC,GPIO_Pin_8);
					GPIO_ResetBits(GPIOB,GPIO_Pin_10);	//yaw停止
					
					TIM_SetCompare1(TIM3,0);
					TIM_SetCompare2(TIM3,0);					//pitch停止
				}
			}
			if(charge_flag)
			{
				pull_work_flag=1;//开始拉线倒回
				GPIO_SetBits(GPIOB,GPIO_Pin_1);
			}
				
			if(launch_flag)
			{
				if(launch_flag_last!=launch_flag)
				{
					launch_times++;
				}
				launchcnt++;
				if(launchcnt>500)
				{
					TIM_SetCompare1(TIM5,2500);//舵机转90度位置,放弹
					TIM_SetCompare2(TIM5,2500);
				}
				if(launchcnt>2500)	//持续2s
				{
					pull_start_time=syscnt;				//记录开始拉线时间
					TIM_SetCompare1(TIM5,1500);//恢复0度位置
					TIM_SetCompare2(TIM5,1500);
					{
						switch (launch_times)
						{
							case 1:
										pull_work_flag=1;
										GPIO_SetBits(GPIOB,GPIO_Pin_1);//PB0给高电平，准备形成开关中断
										launch_flag=0;
										launchcnt=0;
										break;
							
							case 2:		pull_work_flag=1;
										GPIO_SetBits(GPIOB,GPIO_Pin_1);//PB0给高电平，准备形成开关中断
										recharge_work_flag=1;
										launch_flag=0;
										launchcnt=0;
										break;
							
							case 3:		pull_work_flag=1;
										GPIO_SetBits(GPIOB,GPIO_Pin_1);//PB0给高电平，准备形成开关中断
										recharge_work_flag=1;
										launch_flag=0;
										launchcnt=0;
										break;
							
							case 4:		auto_ctrl_flag=1;
										Recharge_Pos_Move(0);//取弹手归位
										launch_flag=0;
										launchcnt=0;
										break;
								
							default:break;

						}
					}
				}
				launch_flag_last=launch_flag;
				
			}
			else
			{
//				TIM_SetCompare1(TIM5,1500);//恢复0度位置
//				TIM_SetCompare2(TIM5,1500);
			}
		}
}
	
/**
  * @brief  摇杆模式
  * @param  None
  * @retval None
  */

 void rc_Control(void)
{
	if(!auto_ctrl_flag)//非自动控制模式
	{
		Gimbal_Power_Flag = 1;//云台给电流
/*----------------------------------遥控器------------------------------------*/
		if(rc_ctrl.rc.s1 == 1)//调试模式
		{
			/*---------------------------各电机位置调整-------------------------------*/		
			pid_gimbal_pos_recharge.SetPoint += (rc_ctrl.rc.ch1 - 1024) * 0.10f;
			pid_gimbal_pos_pushpull1.SetPoint += (rc_ctrl.rc.ch0 - 1024) * 0.20f;
			pid_gimbal_pos_pushpull2.SetPoint += (rc_ctrl.rc.ch0 - 1024) * 0.20f;
		/*-------------------------------Pitch轴----------------------------------*/
			if(rc_ctrl.rc.ch3-1024>300)
			{
				TIM_SetCompare1(TIM3,14399);//调节占空比，改变pitch位置
				TIM_SetCompare2(TIM3,0);
			}
			else if(rc_ctrl.rc.ch3-1024<-300)
			{
				TIM_SetCompare1(TIM3,0);
				TIM_SetCompare2(TIM3,14399);
			}
			else
			{
				TIM_SetCompare1(TIM3,0);
				TIM_SetCompare2(TIM3,0);
			}
			
		/*--------------------------------yaw轴-----------------------------------*/
			
			if(rc_ctrl.rc.ch2-1024>300)
			{
				GPIO_SetBits(GPIOC,GPIO_Pin_8);
				GPIO_ResetBits(GPIOB,GPIO_Pin_10);
			}
			else if(rc_ctrl.rc.ch2-1024<-300)
			{
				GPIO_SetBits(GPIOB,GPIO_Pin_10);
				GPIO_ResetBits(GPIOC,GPIO_Pin_8);
			}
			else
			{
				GPIO_ResetBits(GPIOC,GPIO_Pin_8);
				GPIO_ResetBits(GPIOB,GPIO_Pin_10);
			}
		/*---------------------------发射变量计数调整-----------------------------*/
			first_enter=1;//可以通过摇杆s1重复调整
			
//			//可以重新换弹
//			Recharge_1st=1;
//			Recharge_2nd=1;
//			Recharge_3rd=1;
//			
//			Recharge_1st_Get=0;
//			Recharge_2nd_Get=0;
//			Recharge_3rd_Get=0;
//			launch_times=0;
			
		
			
		}
		/*-------------------换弹参量重置--------------------*/
		
		
		else if(rc_ctrl.rc.s1 == 3)//云台锁定。此时pid要变成最稳定配置
		{
			if(first_enter)
			{
				push_pull_pos_init1=RealPos[0];//记录拉线电机确定好的位置
				push_pull_pos_init2=RealPos[1];//记录拉线电机确定好的位置
				putterpos_init=RealPos[2];//记录换弹电机确定好的位置
				
				first_enter=0;
			}
/*--------------------------拉线、换弹电机锁定---------------------------*/
			pid_gimbal_pos_pushpull1.SetPoint = push_pull_pos_init1;
			pid_gimbal_pos_pushpull2.SetPoint = push_pull_pos_init2;
			pid_gimbal_pos_recharge.SetPoint=RealPos[2];
			//Pid_Gimbal_Pos_Yaw.SetPoint = RealPos[2];

/*-----------------------特定角度转动---------------------------*/
			if(rc_ctrl.rc.ch2-1024>500)
			{
				right_flag=1;
			}
			if(rc_ctrl.rc.ch2-1024<-500)
			{
				left_flag=1;
			}
			

/*----------------------------放弹上弹--------------------------*/
			if(rc_ctrl.rc.ch1 - 1024 > 500)
			{
				launch_flag=1;
			}
			
			
			if(rc_ctrl.rc.ch1 - 1024 <-500)
			{
				charge_flag=1;
			}
			
/*----------------------------小推杆---------------------------*/
			if(rc_ctrl.rc.ch3 - 1024>500)
			{
				Putter_Move_Updown(1);
			}
			else if(rc_ctrl.rc.ch3 - 1024<-500)
			{
				Putter_Move_Updown(0);
			}
			else
			{
//				TIM_SetCompare1(TIM4,0);
//				TIM_SetCompare2(TIM4,0);
			}
			
			
		}
		else if(rc_ctrl.rc.s1 == 2)
		{
			if(rc_ctrl.rc.ch1 - 1024 > 500)
			{
				TIM_SetCompare1(TIM5,2500);//舵机转90度位置,放弹
				TIM_SetCompare2(TIM5,2500);
			}
			else 
			{
				TIM_SetCompare1(TIM5,1500);//舵机恢复位置
				TIM_SetCompare2(TIM5,1500);
			}
		}
}
}

/**
  * @brief  键鼠模式
  * @param  None
  * @retval None
  */
void mouse_key_Control(void)
{
		if(!auto_ctrl_flag)//非自动控制模式
		{
			if(rc_ctrl.key.a==1)
			{	
				left_flag=1;
			}
			if(rc_ctrl.key.d==1)
			{
				right_flag=1;
			}		
			
			if(rc_ctrl.key.s==1)
			{
				charge_flag=1;				//比赛开始，上弹
			}
			
			if(rc_ctrl.key.w==1)
			{
				launch_flag=1;
			}
		}
}
/**
  * @brief  掉电模式
  * @param  None
  * @retval None
  */
void Power_Off(void)
{
	Gimbal_Power_Flag = 0;//云台0电流
	Solenoid_Power_Flag = 0;//电磁阀失能
//	PAout(2) = 0;//光电管失能?有必要失能吗？万一产生电平变化？
//	Motor_Speed_Set(0,1);//履带停转
}
/**
  * @brief  模式执行
  * @param  sta：模式状态
  * @retval None
  */
void statusAct(Status_ sta)
{
		switch(sta.Rc_States)
		{
			case State0:
				mouse_key_Control();
				break;
			case State1:
				rc_Control();
				break;
			case State2:
				Power_Off();
				break;
			default:
				Power_Off();
				break;
		}
}


/**
  * @brief  断线检测
  * @param  Discnect_Flag:断线结构体
  * @retval 返回断线id
  */
u32 offlineDetect(Disconnect_Flag *Discnect_Flag)
{
	u32 Discon_Flag;
/**************CAN1断线检测*******************/
	if(Discnect_Flag->Can1_Connect == 0)
	{
		Discnect_Flag->Can1_Cnt ++;
		Discon_Flag &= 0xfffe;//实时断线标志位清零1111 1111 1111 1110
	}
	if(Discnect_Flag->Can1_Cnt > MS)//断线时间>0.1s
	{
		Discnect_Flag->Can1_Disonnect_Ever = 1;
		Discnect_Flag->Can1_Connect = 1;
		Discon_Flag |= 0x0001;//实时断线标志位置1
	}
/**************CAN2断线检测*******************/
	if(Discnect_Flag->Can2_Connect == 0)
	{
		Discnect_Flag->Can2_Cnt ++;
		Discon_Flag &= 0xfffd;//1111 1111 1111 1101
	}
	if(Discnect_Flag->Can2_Cnt > MS)
	{
		Discnect_Flag->Can2_Disonnect_Ever = 1;
		Discnect_Flag->Can2_Connect = 1;
		Discon_Flag |= 0x0002;
	}
/**************遥控器断线检测*******************/
	if(Discnect_Flag->Rc_Connect == 0)
	{
		Discnect_Flag->Rc_Cnt ++;
		Discon_Flag &= 0xfffb;//1111 1111 1111 1011
	}
	if(Discnect_Flag->Rc_Cnt > MS)
	{
		Discnect_Flag->Rc_Disonnect_Ever = 1;
		Discnect_Flag->Rc_Connect = 1;
		Discon_Flag |= 0x0004;
	}
/**************陀螺仪断线检测*******************/
	if(Discnect_Flag->Gyro_Connect == 0)
	{
		Discnect_Flag->Gyro_Cnt ++;
		Discon_Flag &= 0xfff7;//1111 1111 1111 0111
	}
	if(Discnect_Flag->Gyro_Cnt > MS)
	{
		Discnect_Flag->Gyro_Disonnect_Ever = 1;
		Discnect_Flag->Gyro_Connect = 1;
		Discon_Flag |= 0x0008;
	}
/**************云台断线检测*******************/
	if(Discnect_Flag->Gimbal_Connect == 0)
	{
		Discnect_Flag->Gimbal_Cnt ++;
		Discon_Flag &= 0xffef;//1111 1111 1110 1111
	}
	if(Discnect_Flag->Gimbal_Cnt > MS)
	{
		Discnect_Flag->Gimbal_Disonnect_Ever = 1;
		Discnect_Flag->Gimbal_Connect = 1;
		Discon_Flag |= 0x0010;
	}	
/**************Usart3断线检测*******************/
	if(Discnect_Flag->Usart3_Connect == 0)
	{
		Discnect_Flag->Usart3_Cnt ++;
		Discon_Flag &= 0xffdf;//1111 1111 1101 1111
	}
	if(Discnect_Flag->Usart3_Cnt > MS)
	{
		Discnect_Flag->Usart3_Disonnect_Ever = 1;
		Discnect_Flag->Usart3_Connect = 1;
		Discon_Flag |= 0x0020;
	}
	
	/************裁判系统断线检测**********/
	if(Discnect_Flag->Judgement_Connect == 0)
	{
		Discnect_Flag->Judgement_Cnt ++;
		Discon_Flag &= 0xffbf;//1111 1111 1011 1111
	}
	if(Discnect_Flag->Judgement_Cnt >MS)
	{
		Discnect_Flag->Judgement_Disconnect_Ever = 1;
		Discnect_Flag->Judgement_Connect = 1;
		Discon_Flag |= 0x0040;
	}
	
	
	
	
/*********************************************/
	return Discon_Flag;
}
