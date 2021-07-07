#include "main.h"

/*--------------------------�ڲ�����----------------------------*/
ZeroCheck_Typedef ZeroCheck_Motor[MotorNum];

int Motor_Angle[MotorNum];
float Temp[MotorNum],Hold[MotorNum];
/*--------------------------�ⲿ���ñ���----------------------------*/
extern rmc620_t Gimbal_Motor[MotorNum];
extern float RealPos[MotorNum];
extern float LastPos[MotorNum];
/*--------------------------��������----------------------------*/

/**
  * @brief  λ��ʽ���ٶ�ʽ������
	           Zero->ActualValue ��ʾ�������ǰֵ
						 Zero->LastValue ��ʾ�������һ��ֵ
						 Zero->CountCycle ��ʾ���������ʱԽ��ֵ������������
						 Zero->PreError ��ʾ�������ֵ
						 ʹ�ô˺���ǰҪ������Ӧ������ṹ��� Zero->CountCycle��Zero->LastValue
  * @param  ZeroCheck_Typedef *Zero  ������ṹ��
  *         float value  �������
            short Zerocheck_mode��ȡֵPosition��Speed
  * @retval ȡ����Zerocheck_mode���ֱ�����������λ��ֵ���ٶ�ֵ
  */
float ZeroCheck(ZeroCheck_Typedef *Zero,float value,short Zerocheck_mode)
{
	Zero->ActualValue=value;
	
	Zero->PreError=Zero->ActualValue-Zero->LastValue;
	Zero->LastValue=Zero->ActualValue;
	
	if(Zero->PreError>0.7f*Zero->CountCycle)
	{
		Zero->PreError=Zero->PreError-Zero->CountCycle;
		Zero->Circle++;
	}
	if(Zero->PreError<-0.7f*Zero->CountCycle)
	{
		Zero->PreError=Zero->PreError+Zero->CountCycle;
		Zero->Circle--;
	}
	
	if(Zerocheck_mode==Position)
		return Zero->ActualValue - Zero->Circle*Zero->CountCycle;
	else if(Zerocheck_mode==Speed)
	  return Zero->PreError;
	else 
		return 0;
}


/**
  * @brief  ������ִ�к���
  * @param  None
  * @retval None
  */
void zeroCheck_cal(void)
{
	short i;
	for(i=0;i<MotorNum;i++)
	{
		Hold[i] = (float)Gimbal_Motor[i].angle;
		Temp[i] = ZeroCheck(&ZeroCheck_Motor[i],Hold[i],Position);
		Motor_Angle[i]=(int)Temp[i];
	}

}


/**
  * @brief  ������ṹ�������ʼ��
  * @param  None
  * @retval None
  */
void zeroCheck_init(void)
{
	short i;
	for(i=0;i<MotorNum;i++)
	{
		ZeroCheck_Motor[i].CountCycle=8192;
		ZeroCheck_Motor[i].LastValue=Gimbal_Motor[i].angle;
		ZeroCheck_Motor[i].Circle=0;
	}

}

/**
  * @brief  ���̹�����ֵ
  * @param  None
  * @retval ���̽Ƕȹ���������ֵ
  */

int zeroCheck_motor_pos_output(short i)
{
	switch(i)
	{
		case 0:
			return Motor_Angle[0];
		case 1:
			return Motor_Angle[1];
		case 2:
			return Motor_Angle[2];
		default:
			break;
	}
	return 0;
}
/**
  * @brief  �����⸡�����
  * @param  None
  * @retval None
  */
void zeroCheck_execution(void)
{
	int i = 0;
	for(i = 0;i < MotorNum;i ++)
	{
		LastPos[i] = RealPos[i];
		RealPos[i] = (float)zeroCheck_motor_pos_output(i);//��ȡ��ǰ�������λ��
	}
}
