#ifndef __GIMBALTASK_H
#define __GIMBALTASK_H
//10.21Pos1测定比较合适
#define Pos1 -1023125
#define Pos2 0
#define Pos3 1115412
#define Pos4 2325454
#define PullPos 10000000
#define PutterDelayTime 3300
#define HandlerDelayTime_2nd 500
#define HandlerDelayTime_3rd 2000
void pid_gimbal_Cal_rc(void);
void pid_gimbal_Pos_init(void);
void pid_gimbal_Speed_init(void);
void Recharge_Pos_Move(int position);
void PushPull_Pos_Move(int position);
#endif
