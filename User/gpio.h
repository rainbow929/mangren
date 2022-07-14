/*
 * gpio.h
 *
 *  Created on: Mar 24, 2022
 *      Author: ExplodingONC
 */
#include "timer.h"

//#ifndef USER_GPIO_H_
//#define USER_GPIO_H_

#ifndef __GPIO_H
#define __GPIO_H


#define  Trig_H  GPIO_SetBits(GPIOE,GPIO_Pin_13)
#define  Trig_L  GPIO_ResetBits(GPIOE,GPIO_Pin_13)



#define  Echo_H  GPIO_SetBits(GPIOE,GPIO_Pin_15)
#define  Echo_L  GPIO_ResetBits(GPIOE,GPIO_Pin_15)

#include "debug.h"
#include "stdlib.h"

void Button_INT_Init(void);
void EXTI0_IRQHandler(void);
void ultrasonic_GPIO_Init(void);
void Start_Trig(void);

#endif /* USER_GPIO_H_ */
