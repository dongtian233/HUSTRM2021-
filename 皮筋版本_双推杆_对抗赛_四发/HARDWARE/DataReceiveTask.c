/**********************************************************************************************************
 * @�ļ�     DataReceiveTask.c
 * @˵��     ���ݽ���
 * @�汾  	 V1.0
 * @����     ��־��
 * @����     2021.4
**********************************************************************************************************/
#include "main.h"
unsigned char SaveBuffer[90];
JudgeReceive_t judge_receive;
short F405_DisConnect;
F405_typedef F405;
RM820RReceive_Typedef ChassisMotorCanReceive[4];
ChassisSpeed_t chassis;
extern short Judgement_DisConnect;
extern u8 left_flag,right_flag,launch_flag,charge_flag;
/**********************************************************************************************************
*�� �� ��: Can1Receive0
*����˵��: can1���պ��������յ�����ص��ٶȣ�����ֵ
*��    ��: rx_message0
*�� �� ֵ: ��
**********************************************************************************************************/
void Can1Receive0(CanRxMsg rx_message0)
{
	switch(rx_message0.StdId)
	{ 
		case 0x201:                                                          
				 ChassisMotorCanReceive[0].RealSpeed=rx_message0.Data[2]<<8 | rx_message0.Data[3];
		     ChassisMotorCanReceive[0].Current=rx_message0.Data[4]<<8 | rx_message0.Data[5];
		 break;
		case 0x202:
				 ChassisMotorCanReceive[1].RealSpeed=rx_message0.Data[2]<<8 | rx_message0.Data[3];
		     ChassisMotorCanReceive[1].Current=rx_message0.Data[4]<<8 | rx_message0.Data[5];
		 break;
		case 0x203:
				 ChassisMotorCanReceive[2].RealSpeed=rx_message0.Data[2]<<8 | rx_message0.Data[3];
		     ChassisMotorCanReceive[2].Current=rx_message0.Data[4]<<8 | rx_message0.Data[5];
		 break;
		case 0x204:
				 ChassisMotorCanReceive[3].RealSpeed=rx_message0.Data[2]<<8 | rx_message0.Data[3];
		     ChassisMotorCanReceive[3].Current=rx_message0.Data[4]<<8 | rx_message0.Data[5];
		 break; 
	}
}

/**********************************************************************************************************
*�� �� ��: Can1Receive1
*����˵��: can1���պ���
*��    ��: rx_message1
*�� �� ֵ: ��
**********************************************************************************************************/
//void CAN1_DataReceive(CanRxMsg *rx_message)
//{
//	if(rx_message->StdId == 0x1ff)
//	{
//		 memcpy(&idata.BulletSpeed, &rx_message->Data[2], 2);
//		 memcpy(&idata.SuperPowerLimit, &rx_message->Data[4], 2);
//		 memcpy(&idata.RC_k_100, &rx_message->Data[6], 2);
//		 RC_K = idata.RC_k_100/100.0f;
//	}
//}

/**********************************************************************************************************
*�� �� ��: Can2Receive1
*����˵��: can2���պ���, �����ϲ�崫�ص�xyw���ٶ�
*��    ��: rx_message0
*�� �� ֵ: ��
**********************************************************************************************************/
HeroStatus_typedef	HeroStatus;
void Can2Receive1(CanRxMsg *rx_message)
{
	switch(rx_message->StdId)
	{
		case 0x100:     //�ǵøĹ�����

		  chassis.carSpeedx= (rx_message->Data[0])<<8|rx_message->Data[1];
		  chassis.carSpeedy= (rx_message->Data[2])<<8|rx_message->Data[3];
		  chassis.carSpeedw= (rx_message->Data[4])<<8|rx_message->Data[5];
		  memcpy(&HeroStatus.ChassisMode, &rx_message->Data[6], 1);
		 
		 
		 chassis.carSpeedx = LIMIT_MAX_MIN(chassis.carSpeedx*3,4000,-4000);
		 chassis.carSpeedy = LIMIT_MAX_MIN(chassis.carSpeedy*3,4000,-4000);
//   chassis.carSpeedw = LIMIT_MAX_MIN(chassis.carSpeedw,2000,-2000);
		F405.SuperPowerLimit=rx_message->Data[7];
			F405_DisConnect=0; 
		break;
		case 0x121:
			F405.Armortype= rx_message->Data[0];
			
	}
}

/**********************************************************************************************************
*�� �� ��: JudgeBuffReceive
*����˵��: ����ϵͳ���պ���
*��    ��: ReceiveBuffer[]  DataLen
*�� �� ֵ: ��
**********************************************************************************************************/
void JudgeBuffReceive(unsigned char ReceiveBuffer[],uint16_t DataLen)
{
	uint16_t cmd_id;
	short PackPoint;
	memcpy(&SaveBuffer[JudgeBufBiggestSize],&ReceiveBuffer[0],JudgeBufBiggestSize);		//��ReceiveBuffer[0]��ַ������SaveBuffer[45], ���ο���45��, ����һ�ν��յĴ浽�����
	for(PackPoint=0;PackPoint<JudgeBufBiggestSize;PackPoint++)		//�ȴ���ǰ�������(����һ�����ѽ������)
	{
		if(SaveBuffer[PackPoint]==0xA5) 
		{	
			if((Verify_CRC8_Check_Sum(&SaveBuffer[PackPoint],5)==1))		//frame_header ��λ����У��
			{
				cmd_id=(SaveBuffer[PackPoint+6])&0xff;
				cmd_id=(cmd_id<<8)|SaveBuffer[PackPoint+5];  
				DataLen=SaveBuffer[PackPoint+2]&0xff;
				DataLen=(DataLen<<8)|SaveBuffer[PackPoint+1];
				
				
				
				if((cmd_id==0x0001)&&(Verify_CRC16_Check_Sum(&SaveBuffer[PackPoint],DataLen+9))) 
				{
					judge_receive.GameStage=(SaveBuffer[PackPoint+7+0]&(0xf0))>>4;
				}
				//������״̬����
				if((cmd_id==0x0201)&&(Verify_CRC16_Check_Sum(&SaveBuffer[PackPoint],DataLen+9))) 
				{
					judge_receive.robot_id=SaveBuffer[PackPoint+7+0];  //  3 4 5 �췽������  103 104 105 ����������
					judge_receive.RobotLevel=SaveBuffer[PackPoint+7+1];
					judge_receive.remainHP=(SaveBuffer[PackPoint+7+3]<<8)|SaveBuffer[PackPoint+7+2]; 
					judge_receive.maxHP=(SaveBuffer[PackPoint+7+5]<<8)|SaveBuffer[PackPoint+7+4]; 
					judge_receive.HeatCool17=(SaveBuffer[PackPoint+7+7]<<8)|SaveBuffer[PackPoint+7+6];
					judge_receive.HeatMax17=(SaveBuffer[PackPoint+7+9]<<8)|SaveBuffer[PackPoint+7+8];
					judge_receive.BulletSpeedMax17=(SaveBuffer[PackPoint+7+11]<<8)|SaveBuffer[PackPoint+7+10];
				
					judge_receive.HeatCool42=(SaveBuffer[PackPoint+7+19]<<8)|SaveBuffer[PackPoint+7+18];
					judge_receive.BulletSpeedMax42=(SaveBuffer[PackPoint+7+21]<<8|SaveBuffer[PackPoint+7+20]);
					judge_receive.HeatMax42=(SaveBuffer[PackPoint+7+21]<<8)|SaveBuffer[PackPoint+7+20];
					judge_receive.BulletSpeedMax42=(SaveBuffer[PackPoint+7+23]<<8)|SaveBuffer[PackPoint+7+22];
					judge_receive.MaxPower=(SaveBuffer[PackPoint+7+25]<<8)|SaveBuffer[PackPoint+7+24];
				}
					
				//ʵʱ������������
				if((cmd_id==0x0202)&&(Verify_CRC16_Check_Sum(&SaveBuffer[PackPoint],DataLen+9)))
				{
					memcpy(&judge_receive.realChassisOutV,&SaveBuffer[PackPoint+7+0],2);
					judge_receive.realChassisOutV = judge_receive.realChassisOutV /1000.0f;
					memcpy(&judge_receive.realChassisOutA,&SaveBuffer[PackPoint+7+2],2);
					memcpy(&judge_receive.realChassispower,&SaveBuffer[PackPoint+7+4],4);
					memcpy(&judge_receive.remainEnergy,&SaveBuffer[PackPoint+7+8],2);
					memcpy(&judge_receive.shooterHeat17,&SaveBuffer[PackPoint+7+10],2);
					memcpy(&judge_receive.shooterHeat42,&SaveBuffer[PackPoint+7+14],2);
				}
				
				//����������
				if((cmd_id==0x0204)&&(Verify_CRC16_Check_Sum(&SaveBuffer[PackPoint],DataLen+9)))
				{
					judge_receive.RobotBuffState = SaveBuffer[PackPoint+7];
					judge_receive.HPBuff = judge_receive.RobotBuffState & 0x01;
					judge_receive.CoolBuff = (judge_receive.RobotBuffState & 0x02) >> 1;
					judge_receive.DefenceBuff = (judge_receive.RobotBuffState & 0x04) >> 2;
					judge_receive.AttackBuff = (judge_receive.RobotBuffState & 0x08) >> 3;
				}
				
				//ʵʱ�����Ϣ
					if((cmd_id==0x0207)&&(Verify_CRC16_Check_Sum(&SaveBuffer[PackPoint],DataLen+9)))
					{
//					if((SaveBuffer[PackPoint+7+0]==1) && (SaveBuffer[PackPoint+7+1]) == 1)
//					{
						judge_receive.bulletFreq = SaveBuffer[PackPoint+7+2];
						memcpy(&judge_receive.bulletSpeed,&SaveBuffer[PackPoint+7+3],4);
						judge_receive.ShootCpltFlag = 1;//��־λ��˵�����յ�����ϵͳ����
//					}
					}
				
					//������ڷ��ڷ�������м���	���ڿ�����Ϣ  ����ͨѶ
//					if((cmd_id==0x0302)&&(Verify_CRC16_Check_Sum(&SaveBuffer[PackPoint],DataLen+9)))
//					{
//						//���ͷ���ͬ������˻��ͻ��� 6��106�����շ��Ǳ������ڷ����8��108
////						if((SaveBuffer[PackPoint+7+2]<<8|SaveBuffer[PackPoint+7+3]==(judge_receive.robot_id-2))
////						 &&(SaveBuffer[PackPoint+7+4]<<8|SaveBuffer[PackPoint+7+5]==judge_receive.robot_id))
//						{
//							switch (SaveBuffer[PackPoint+7+2])
//							{
//								case 0x01:
//									left_flag=1;
//									break;
//								case 0x02:
//									right_flag=1;
//									break;
//								case 0x04:
//									launch_flag=1;
//									break;
//								case 0x08:
//									charge_flag=1;
//									break;
//								default:
//									break; 
//							}
//						}
//					}
				
				
				//���ڻ����˿ͻ���ָ����
				if((cmd_id==0x020A)&&(Verify_CRC16_Check_Sum(&SaveBuffer[PackPoint],DataLen+9)))
				{
					judge_receive.dart_launch_opening_status=SaveBuffer[PackPoint+7];
					judge_receive.dart_attack_target=SaveBuffer[PackPoint+7+1];
					judge_receive.target_change_time=(SaveBuffer[PackPoint+7+3]<<8)|SaveBuffer[PackPoint+7+2];
					judge_receive.operate_launch_cmd_time=(SaveBuffer[PackPoint+7+5]<<8)|SaveBuffer[PackPoint+7+4];
				}
			}
		}
		
//	Judgement_DisConnect =0;
	memcpy(&SaveBuffer[0],&SaveBuffer[JudgeBufBiggestSize],JudgeBufBiggestSize);		//��SaveBuffer[24]��ַ������SaveBuffer[0], ���ο���24������֮ǰ�浽����������ᵽǰ�棬׼������
	}
	Discnect_Flag.Judgement_Cnt=0;
	Discnect_Flag.Judgement_Connect=0;
}

/**********************************************************************************************************
*�� �� ��: F405_Rst
*����˵��: ���ذ����ִ�к���
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void F405_Rst(void)
{
	chassis.carSpeedx = 0;
	chassis.carSpeedy = 0;
	chassis.carSpeedw = 0;
}



