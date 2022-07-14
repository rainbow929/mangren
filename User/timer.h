#ifndef __TIMER_H
#define __TIMER_H

#include "debug.h"
#include "stdlib.h"
#include "ch32v30x_conf.h"

#define  fengmingqi1_H  GPIO_SetBits(GPIOD,GPIO_Pin_10)
#define  fengmingqi1_L  GPIO_ResetBits(GPIOD,GPIO_Pin_10)


void Tick_TIM_Init( u16 arr, u16 psc);
void Refresh_TIM_Init( u16 arr, u16 psc);

void Tick_TIM_INT_Init(void);
void Refresh_TIM_INT_Init(void);

void TIM6_IRQHandler(void); // tick timer
void TIM7_IRQHandler(void); // refresh timer

void Input_Capture_Init( u16 arr, u16 psc );
void ENABLE_TIM(void);
void DISABLE_TIM(void);
u32 GetCount(void);
float Ultrasoniclength(void);

#endif /* USER_TIMER_H_ */
