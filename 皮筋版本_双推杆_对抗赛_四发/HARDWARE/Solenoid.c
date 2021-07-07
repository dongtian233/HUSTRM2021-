/*---------------------------------------------------------------------------*/
/** 
  **@brief 换弹、倒回功能配置
  **
  **/
/*---------------------------------------------------------------------------*/



#include "main.h"
u8 Solenoid_Power_Flag = 1;
u8 Recharge_1st=1,Recharge_2nd=1,Recharge_3rd=1;//标志是在换上第几个飞镖
u8 Recharge_1st_Get=0,Recharge_2nd_Get=0,Recharge_3rd_Get=0;
u8 recharge_work_flag=0 ;//标志换弹开始结束，换弹结束是0，可以开始是1

extern u8 left_flag;
extern u8 right_flag;
extern u8 launch_flag;
extern u8 charge_flag;
u8 pull_work_flag=0;//拉线电机工作标志位，0：不动；1：倒回
u8 push_work_flag=0;//推线，同上
int delay = 1000;
u32 PutterDelay=0;
u32 HandlerDelay_2nd=0;
u32 HandlerDelay_3rd=0;

u32 time=0;
extern int push_pull_pos_init1;
extern u8 charge_flag;
extern u8 auto_ctrl_flag;
extern int putterpos_init;
extern int push_pull_pos_init1;
extern int push_pull_pos_init2;
extern float RealPos[MotorNum];
extern float RealSpeed[MotorNum];

/**
  * @brief  倒回动作
  * @param  None
  * @retval None
  */
void pullworkTask()
{
		if(pull_work_flag)
		{
				auto_ctrl_flag=1;
				PushPull_Pos_Move(PullPos);
		}
		//向回拉线
		
		else if(push_work_flag)//进中断后进入
		{
			PushPull_Pos_Move(0);//拉线电机恢复位置
			if(Range_In(1000,RealPos[0],push_pull_pos_init1))	//PushPullPosInit 拉线电机1初始化值
			{
				push_work_flag=0;//停止推线
//				auto_ctrl_flag=0;//无换弹测试用
				if(charge_flag)
				{
					charge_flag=0;//比赛前的装弹结束
					auto_ctrl_flag=0;	//回归手动控制
					
				}
				else if(recharge_work_flag==0)		//不是装弹，但已经完成换弹
				{
					auto_ctrl_flag=0;
				}
					
			}
		}
		else
		{
			
		}
		
	}





/**
  * @brief  换弹动作
  * @param  None
  * @retval None
  */
void rechargeTask(void)
{
	if(recharge_work_flag)
	{	
		if(Recharge_1st)//第一次换弹是单程，后两次是往返双程。
		{
			Recharge_Pos_Move(Pos1);//换弹电机位置环直接向下移动
			if(Range_In(1000,RealPos[2],putterpos_init+Pos1)
				&&Range_In(1000,RealPos[0],push_pull_pos_init1))//现在位置和目标位置角度相差很小，说明换弹完成
			{
				recharge_work_flag = 0;//换弹停止工作
				Recharge_1st = 0;
				auto_ctrl_flag=0;
			}
		}
		else if(Recharge_2nd)
		{
			if(Recharge_2nd_Get == 0)//没有移动到指定位置
			{
				Recharge_Pos_Move(Pos3);
				Recharge_2nd_Get = Range_In(1000,RealPos[2],putterpos_init+Pos3);//位置移动完成返回1
			}
			else
			{
				if(PutterDelay<PutterDelayTime)//推杆电机还没放下飞镖
				{
					PutterDelay++;
					Putter_Move_Updown(0);
				}
				else
				{
					Recharge_Pos_Move(Pos1);//取弹手归位
					{
						if(HandlerDelay_2nd<HandlerDelayTime_2nd)
						{
							HandlerDelay_2nd++;
						}
						else
						{
							Putter_Move_Updown(1);//支架上移,便于自动发射
						}
					}
					if(Range_In(1000,RealPos[2],putterpos_init+Pos1))//电机移动换弹完成
					{
						PutterDelay=0;
						HandlerDelay_2nd=0;
						Recharge_2nd = 0;
						Putter_Move_Updown(1);//支架上移
						recharge_work_flag=0;
//						if(Range_In(1000,RealPos[0],push_pull_pos_init1))	//第二次换弹等待推线电机就位
//						{
//							auto_ctrl_flag=0;
//							recharge_work_flag = 0;//换弹停止工作

//						}
					}
				}
			}
		}
		
		else if(Recharge_3rd)
		{
			if(Recharge_3rd_Get == 0)
			{
				Recharge_Pos_Move(Pos4);
				Recharge_3rd_Get = Range_In(1000,RealPos[2],putterpos_init+Pos4);//位置移动完成返回1
			}
			else
			{
				if(PutterDelay<PutterDelayTime)
				{
					PutterDelay++;
					Putter_Move_Updown(0);
				}
				else
				{
					Recharge_Pos_Move(Pos1);
					if(HandlerDelay_3rd<HandlerDelayTime_3rd)
					{
						HandlerDelay_3rd++;
					}
					else
					{
						Putter_Move_Updown(1);
					}
					if(Range_In(1000,RealPos[2],putterpos_init+Pos1))//电机移动换弹完成
					{
						PutterDelay=0;
						HandlerDelay_3rd=0;
						Recharge_3rd = 0;
						Putter_Move_Updown(1);
						recharge_work_flag=0;
//						if(Range_In(1000,RealPos[0],push_pull_pos_init1))	//第三次换弹等待推线电机就位
//						{
//							auto_ctrl_flag=0;
//							recharge_work_flag = 0;//换弹停止工作

//						}
					}
				}
			}
		}
	}
}







void flag_Initialize()
{
	auto_ctrl_flag=0;
	push_work_flag=0;
	pull_work_flag=0;
	recharge_work_flag=0;
	charge_flag=0;
	left_flag=0;
	right_flag=0;
	launch_flag=0;
	
}


/**
  * @brief  范围判定函数
  * @param  Range:单边范围长度
  *         Actual：实际值
  *		    Target:目标值
  * @retval 1:进入范围;0:不在范围内
  */
int Range_In(float Range,float Actual,float Target)
{
	if(ABS(Actual-Target) <= Range)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

