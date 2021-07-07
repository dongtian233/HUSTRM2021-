#ifndef __ZEROCHECK_H_
#define __ZEROCHECK_H_

#define Position 1
#define Speed    2

typedef struct 
{
	float Circle;           //ת��Ȧ��
	float CountCycle;       //ת��һȦ���ܼ�������
	float LastValue;        //����������һ�ε�ֵ	
	float ActualValue;      //����������ǰֵ
	float PreError;         //������жϲ�ֵ
}
ZeroCheck_Typedef;

float ZeroCheck(ZeroCheck_Typedef *Zero,float value,short Zerocheck_mode);
void zeroCheck_cal(void);
void zeroCheck_init(void);
int zeroCheck_motor_pos_output(short i);
void zeroCheck_execution(void);

#endif
