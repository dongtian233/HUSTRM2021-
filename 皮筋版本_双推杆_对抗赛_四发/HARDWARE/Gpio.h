#ifndef __GPIO_H__
#define __GPIO_H__

#include "stm32f10x.h"
void gpio_exti_Config(void);
void	yaw_gpio_Config(void);
void GPIO_Photocell(void);
u8 Get_Photocell_1(void);
u32 Photocell_Detect(void);
#endif
