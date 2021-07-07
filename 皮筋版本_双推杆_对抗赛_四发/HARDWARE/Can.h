#ifndef __CAN_H
#define __CAN_H

#include "main.h"

void can1_Config(void);
void gimbalCurrent_send(int a, int b, int c, int d);
float Int_To_Float(int Inte);
void CAN2_Config(void);
int Float_To_Int(float Floa);

#endif
