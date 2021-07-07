/*---------------------------------------------------------------------------*/
/** 
  **@brief ���������ع�������
  **
  **/
/*---------------------------------------------------------------------------*/



#include "main.h"
u8 Solenoid_Power_Flag = 1;
u8 Recharge_1st=1,Recharge_2nd=1,Recharge_3rd=1;//��־���ڻ��ϵڼ�������
u8 Recharge_1st_Get=0,Recharge_2nd_Get=0,Recharge_3rd_Get=0;
u8 recharge_work_flag=0 ;//��־������ʼ����������������0�����Կ�ʼ��1

extern u8 left_flag;
extern u8 right_flag;
extern u8 launch_flag;
extern u8 charge_flag;
u8 pull_work_flag=0;//���ߵ��������־λ��0��������1������
u8 push_work_flag=0;//���ߣ�ͬ��
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
  * @brief  ���ض���
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
		//�������
		
		else if(push_work_flag)//���жϺ����
		{
			PushPull_Pos_Move(0);//���ߵ���ָ�λ��
			if(Range_In(1000,RealPos[0],push_pull_pos_init1))	//PushPullPosInit ���ߵ��1��ʼ��ֵ
			{
				push_work_flag=0;//ֹͣ����
//				auto_ctrl_flag=0;//�޻���������
				if(charge_flag)
				{
					charge_flag=0;//����ǰ��װ������
					auto_ctrl_flag=0;	//�ع��ֶ�����
					
				}
				else if(recharge_work_flag==0)		//����װ�������Ѿ���ɻ���
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
  * @brief  ��������
  * @param  None
  * @retval None
  */
void rechargeTask(void)
{
	if(recharge_work_flag)
	{	
		if(Recharge_1st)//��һ�λ����ǵ��̣�������������˫�̡�
		{
			Recharge_Pos_Move(Pos1);//�������λ�û�ֱ�������ƶ�
			if(Range_In(1000,RealPos[2],putterpos_init+Pos1)
				&&Range_In(1000,RealPos[0],push_pull_pos_init1))//����λ�ú�Ŀ��λ�ýǶ�����С��˵���������
			{
				recharge_work_flag = 0;//����ֹͣ����
				Recharge_1st = 0;
				auto_ctrl_flag=0;
			}
		}
		else if(Recharge_2nd)
		{
			if(Recharge_2nd_Get == 0)//û���ƶ���ָ��λ��
			{
				Recharge_Pos_Move(Pos3);
				Recharge_2nd_Get = Range_In(1000,RealPos[2],putterpos_init+Pos3);//λ���ƶ���ɷ���1
			}
			else
			{
				if(PutterDelay<PutterDelayTime)//�Ƹ˵����û���·���
				{
					PutterDelay++;
					Putter_Move_Updown(0);
				}
				else
				{
					Recharge_Pos_Move(Pos1);//ȡ���ֹ�λ
					{
						if(HandlerDelay_2nd<HandlerDelayTime_2nd)
						{
							HandlerDelay_2nd++;
						}
						else
						{
							Putter_Move_Updown(1);//֧������,�����Զ�����
						}
					}
					if(Range_In(1000,RealPos[2],putterpos_init+Pos1))//����ƶ��������
					{
						PutterDelay=0;
						HandlerDelay_2nd=0;
						Recharge_2nd = 0;
						Putter_Move_Updown(1);//֧������
						recharge_work_flag=0;
//						if(Range_In(1000,RealPos[0],push_pull_pos_init1))	//�ڶ��λ����ȴ����ߵ����λ
//						{
//							auto_ctrl_flag=0;
//							recharge_work_flag = 0;//����ֹͣ����

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
				Recharge_3rd_Get = Range_In(1000,RealPos[2],putterpos_init+Pos4);//λ���ƶ���ɷ���1
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
					if(Range_In(1000,RealPos[2],putterpos_init+Pos1))//����ƶ��������
					{
						PutterDelay=0;
						HandlerDelay_3rd=0;
						Recharge_3rd = 0;
						Putter_Move_Updown(1);
						recharge_work_flag=0;
//						if(Range_In(1000,RealPos[0],push_pull_pos_init1))	//�����λ����ȴ����ߵ����λ
//						{
//							auto_ctrl_flag=0;
//							recharge_work_flag = 0;//����ֹͣ����

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
  * @brief  ��Χ�ж�����
  * @param  Range:���߷�Χ����
  *         Actual��ʵ��ֵ
  *		    Target:Ŀ��ֵ
  * @retval 1:���뷶Χ;0:���ڷ�Χ��
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

