#ifndef __DATARECEIVETASK_H
#define __DATARECEIVETASK_H
#include "main.h"

typedef struct{
	short SuperPowerLimit;
	short Chassis_Flag;
	short Armortype;
}F405_typedef;

#define BigArmor 1
#define SmallArmor 0

typedef struct{
	short Angle;
	short RealSpeed;  
  short Current;	
}RM820RReceive_Typedef;

typedef struct 
{
  short carSpeedx;
	short carSpeedy;
	short carSpeedw;
} ChassisSpeed_t;

typedef struct
{
  char HeatUpdate_NoUpdate;
	char SpeedUpdate_NoUpdate;
	
	//0x0201
	uint8_t robot_id;
	uint8_t RobotLevel;
	uint16_t remainHP;
	uint16_t maxHP;
	uint16_t HeatCool17;		//17mmǹ��ÿ����ȴֵ
	uint16_t HeatMax17;			//17mmǹ����������
	uint16_t BulletSpeedMax17;	//17mmǹ�������ٶ�
	uint16_t MaxPower;			//���̹�����������

	//0x0202
	uint16_t realChassisOutV;
	uint16_t realChassisOutA;
	float realChassispower;
	uint16_t remainEnergy;       //ʣ������
	short shooterHeat17;
	short shooterHeat42;
	
	//0x0204
	uint8_t RobotBuffState;
	//��1��ʾ�и�buff
	uint8_t HPBuff;
	uint8_t CoolBuff;
	uint8_t DefenceBuff;
	uint8_t AttackBuff;
	
	//0x0207
	uint8_t bulletFreq;		//���Ƶ��
	uint8_t ShootCpltFlag; //�����һ���ӵ���־λ
	
	//flag
	short HeatUpdateFlag;	
	
	//not in use
	uint8_t cardType;
	uint8_t CardIdx;
	uint16_t HeatCool42;		//42mmǹ��ÿ����ȴֵ
	uint16_t HeatMax42;			//42mmǹ����������
	uint16_t BulletSpeedMax42;	//42mmǹ�������ٶ�
	
	short  HeatTop;					//����õ�����������
	uint16_t LastShooterHeat42;
	uint16_t LastShooterHeat17;

	float bulletSpeed;		//��ǰ����
	float LastbulletSpeed;
	
	float bulletSpeed17;
	float bulletSpeed42;
	float LastbulletSpeed17;
	
	//0x0001:
	uint8_t GameStage;
	
	//0x20A:
	uint8_t dart_launch_opening_status;
	uint8_t dart_attack_target;
	uint16_t target_change_time;
	uint16_t operate_launch_cmd_time;
	
}
JudgeReceive_t;

typedef enum
{
    Chassis_Act_Mode			=	0x001,
    Chassis_SelfProtect_Mode  =	0x002,
    Chassis_CeaseMode   	=	0x000,
    Chassis_Single_Mode		= 0x003
} ChassisMode_typedef;

typedef struct
{
    ChassisMode_typedef 	ChassisMode;

} HeroStatus_typedef;

void Can1Receive0(CanRxMsg rx_message0);
//void Can2Receive0(CanRxMsg *rx_message);
void Can2Receive1(CanRxMsg *rx_message);

void JudgeBuffReceive(unsigned char ReceiveBuffer[],uint16_t DataLen);

void F405_Rst(void);



#endif
