/*---------------------------------------------------------------------------*/
/** 
  **@brief ϵͳʶ�𡢿���
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
int pitch_pulse_set;//0~14400��Ӧռ�ձ�
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
 **@brief ���ݲ���ϵͳ�ź����ɿ����ź�
 **@date 2021.6
 **/
void judgementAct(JudgeReceive_t judge)
{
	if(judge.GameStage==4)											//����������
	{
	if(judge.dart_launch_opening_status==0)							//բ�Ÿոտ���
	{
		if(first_emission)
		{
			launch_flag=1;		
			first_emission=0;			
		}
	}
//բ�Źر�״̬
	if(judge.dart_launch_opening_status==1)
	{
		first_emission=1;						//����ʹ�ܷ���
		if(first_close_gate)					//�����е�һ�ι�բ��
																					
		{
			charge_flag=1;						//�ϵ�
//			left_flag=1;						//�Զ���׼����ǰ��վ
			first_close_gate=0;
		}
	}
	//ԭ��ǰ��վ��Ŀ������
//	if(dart_attack_target_last==0&&judge.dart_attack_target==1)
//	{
//		right_flag=1;
//	}

	dart_attack_target_last=judge.dart_attack_target;
	dart_launch_opening_status_last=judge.dart_launch_opening_status;
	operate_launch_cmd_time_last=judge.operate_launch_cmd_time;//�в������ӳ٣���ʹ��
	}

	
}
/**
 **@brief ���ݸ�flag�ź�ʵʩ����
 **@retval None
 **/
void judge_pc_Control(void)
{
	if(!auto_ctrl_flag)
	{//Ĭ����׼ǰ��վ����˲�����ת
			if(left_flag)		//��׼ǰ��վ
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
					GPIO_ResetBits(GPIOB,GPIO_Pin_10);	//yawֹͣ
				}
				else
				{
					GPIO_ResetBits(GPIOC,GPIO_Pin_8);
					GPIO_ResetBits(GPIOB,GPIO_Pin_10);	//yawֹͣ
				}
				
				if(cntpitch_left<Pitch_Down_Time)
				{
					TIM_SetCompare1(TIM3,14399);//����ռ�ձȣ��ı�pitchλ��
					TIM_SetCompare2(TIM3,0);
					cntpitch_left++;
				}
				else
				{
					TIM_SetCompare1(TIM3,0);
					TIM_SetCompare2(TIM3,0);					//pitchֹͣ
				}
				if((cntpitch_left==Pitch_Down_Time)&&(cntyaw_left==Yaw_Left_Time))
				{
					left_flag=0;
					
					GPIO_ResetBits(GPIOC,GPIO_Pin_8);
					GPIO_ResetBits(GPIOB,GPIO_Pin_10);	//yawֹͣ
					
					TIM_SetCompare1(TIM3,0);
					TIM_SetCompare2(TIM3,0);					//pitchֹͣ
				}
			}
			
			if(right_flag)		//��׼����
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
					GPIO_ResetBits(GPIOB,GPIO_Pin_10);	//yawֹͣ
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
					TIM_SetCompare2(TIM3,0);					//pitchֹͣ
				}
				if(cntpitch_right==(Pitch_Up_Time-Pitch_Down_Time)&&cntyaw_right==(Yaw_Right_Time-Yaw_Left_Time))
				{
					right_flag =0;
					
					GPIO_ResetBits(GPIOC,GPIO_Pin_8);
					GPIO_ResetBits(GPIOB,GPIO_Pin_10);	//yawֹͣ
					
					TIM_SetCompare1(TIM3,0);
					TIM_SetCompare2(TIM3,0);					//pitchֹͣ
				}
			}
			if(charge_flag)
			{
				pull_work_flag=1;//��ʼ���ߵ���
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
					TIM_SetCompare1(TIM5,2500);//���ת90��λ��,�ŵ�
					TIM_SetCompare2(TIM5,2500);
				}
				if(launchcnt>2500)	//����2s
				{
					pull_start_time=syscnt;				//��¼��ʼ����ʱ��
					TIM_SetCompare1(TIM5,1500);//�ָ�0��λ��
					TIM_SetCompare2(TIM5,1500);
					{
						switch (launch_times)
						{
							case 1:
										pull_work_flag=1;
										GPIO_SetBits(GPIOB,GPIO_Pin_1);//PB0���ߵ�ƽ��׼���γɿ����ж�
										launch_flag=0;
										launchcnt=0;
										break;
							
							case 2:		pull_work_flag=1;
										GPIO_SetBits(GPIOB,GPIO_Pin_1);//PB0���ߵ�ƽ��׼���γɿ����ж�
										recharge_work_flag=1;
										launch_flag=0;
										launchcnt=0;
										break;
							
							case 3:		pull_work_flag=1;
										GPIO_SetBits(GPIOB,GPIO_Pin_1);//PB0���ߵ�ƽ��׼���γɿ����ж�
										recharge_work_flag=1;
										launch_flag=0;
										launchcnt=0;
										break;
							
							case 4:		auto_ctrl_flag=1;
										Recharge_Pos_Move(0);//ȡ���ֹ�λ
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
//				TIM_SetCompare1(TIM5,1500);//�ָ�0��λ��
//				TIM_SetCompare2(TIM5,1500);
			}
		}
}
	
/**
  * @brief  ҡ��ģʽ
  * @param  None
  * @retval None
  */

 void rc_Control(void)
{
	if(!auto_ctrl_flag)//���Զ�����ģʽ
	{
		Gimbal_Power_Flag = 1;//��̨������
/*----------------------------------ң����------------------------------------*/
		if(rc_ctrl.rc.s1 == 1)//����ģʽ
		{
			/*---------------------------�����λ�õ���-------------------------------*/		
			pid_gimbal_pos_recharge.SetPoint += (rc_ctrl.rc.ch1 - 1024) * 0.10f;
			pid_gimbal_pos_pushpull1.SetPoint += (rc_ctrl.rc.ch0 - 1024) * 0.20f;
			pid_gimbal_pos_pushpull2.SetPoint += (rc_ctrl.rc.ch0 - 1024) * 0.20f;
		/*-------------------------------Pitch��----------------------------------*/
			if(rc_ctrl.rc.ch3-1024>300)
			{
				TIM_SetCompare1(TIM3,14399);//����ռ�ձȣ��ı�pitchλ��
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
			
		/*--------------------------------yaw��-----------------------------------*/
			
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
		/*---------------------------���������������-----------------------------*/
			first_enter=1;//����ͨ��ҡ��s1�ظ�����
			
//			//�������»���
//			Recharge_1st=1;
//			Recharge_2nd=1;
//			Recharge_3rd=1;
//			
//			Recharge_1st_Get=0;
//			Recharge_2nd_Get=0;
//			Recharge_3rd_Get=0;
//			launch_times=0;
			
		
			
		}
		/*-------------------������������--------------------*/
		
		
		else if(rc_ctrl.rc.s1 == 3)//��̨��������ʱpidҪ������ȶ�����
		{
			if(first_enter)
			{
				push_pull_pos_init1=RealPos[0];//��¼���ߵ��ȷ���õ�λ��
				push_pull_pos_init2=RealPos[1];//��¼���ߵ��ȷ���õ�λ��
				putterpos_init=RealPos[2];//��¼�������ȷ���õ�λ��
				
				first_enter=0;
			}
/*--------------------------���ߡ������������---------------------------*/
			pid_gimbal_pos_pushpull1.SetPoint = push_pull_pos_init1;
			pid_gimbal_pos_pushpull2.SetPoint = push_pull_pos_init2;
			pid_gimbal_pos_recharge.SetPoint=RealPos[2];
			//Pid_Gimbal_Pos_Yaw.SetPoint = RealPos[2];

/*-----------------------�ض��Ƕ�ת��---------------------------*/
			if(rc_ctrl.rc.ch2-1024>500)
			{
				right_flag=1;
			}
			if(rc_ctrl.rc.ch2-1024<-500)
			{
				left_flag=1;
			}
			

/*----------------------------�ŵ��ϵ�--------------------------*/
			if(rc_ctrl.rc.ch1 - 1024 > 500)
			{
				launch_flag=1;
			}
			
			
			if(rc_ctrl.rc.ch1 - 1024 <-500)
			{
				charge_flag=1;
			}
			
/*----------------------------С�Ƹ�---------------------------*/
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
				TIM_SetCompare1(TIM5,2500);//���ת90��λ��,�ŵ�
				TIM_SetCompare2(TIM5,2500);
			}
			else 
			{
				TIM_SetCompare1(TIM5,1500);//����ָ�λ��
				TIM_SetCompare2(TIM5,1500);
			}
		}
}
}

/**
  * @brief  ����ģʽ
  * @param  None
  * @retval None
  */
void mouse_key_Control(void)
{
		if(!auto_ctrl_flag)//���Զ�����ģʽ
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
				charge_flag=1;				//������ʼ���ϵ�
			}
			
			if(rc_ctrl.key.w==1)
			{
				launch_flag=1;
			}
		}
}
/**
  * @brief  ����ģʽ
  * @param  None
  * @retval None
  */
void Power_Off(void)
{
	Gimbal_Power_Flag = 0;//��̨0����
	Solenoid_Power_Flag = 0;//��ŷ�ʧ��
//	PAout(2) = 0;//����ʧ��?�б�Ҫʧ������һ������ƽ�仯��
//	Motor_Speed_Set(0,1);//�Ĵ�ͣת
}
/**
  * @brief  ģʽִ��
  * @param  sta��ģʽ״̬
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
  * @brief  ���߼��
  * @param  Discnect_Flag:���߽ṹ��
  * @retval ���ض���id
  */
u32 offlineDetect(Disconnect_Flag *Discnect_Flag)
{
	u32 Discon_Flag;
/**************CAN1���߼��*******************/
	if(Discnect_Flag->Can1_Connect == 0)
	{
		Discnect_Flag->Can1_Cnt ++;
		Discon_Flag &= 0xfffe;//ʵʱ���߱�־λ����1111 1111 1111 1110
	}
	if(Discnect_Flag->Can1_Cnt > MS)//����ʱ��>0.1s
	{
		Discnect_Flag->Can1_Disonnect_Ever = 1;
		Discnect_Flag->Can1_Connect = 1;
		Discon_Flag |= 0x0001;//ʵʱ���߱�־λ��1
	}
/**************CAN2���߼��*******************/
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
/**************ң�������߼��*******************/
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
/**************�����Ƕ��߼��*******************/
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
/**************��̨���߼��*******************/
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
/**************Usart3���߼��*******************/
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
	
	/************����ϵͳ���߼��**********/
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
