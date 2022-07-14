/*
 * timer.c
 *
 *  Created on: Mar 22, 2022
 *      Author: ExplodingONC
 */
#include "timer.h"
#include "gpio.h"
#include "debug.h"
#include "lcd_st7789.h"

#define TRUE 1
#define FALSE 0

float distance=0;
u16 count = 0;

void Input_Capture_Init( u16 arr, u16 psc )

{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    NVIC_InitTypeDef        NVIC_InitStructure;

    //使能IM2时钟
    RCC_APB1PeriphClockCmd( RCC_APB1Periph_TIM2, ENABLE);

    Button_INT_Init();
    TIM_DeInit(TIM2);

    //定时器周期，实际就是设定自动重载寄存器 ARR 的值， ARR 为要装载到实际自动重载寄存器（即影子寄存器） 的值， 可设置范围为 0 至 65535。
    TIM_TimeBaseInitStructure.TIM_Period = arr;

    //定时器预分频器设置，时钟源经该预分频器才是定时器计数时钟CK_CNT，它设定 PSC 寄存器的值。
    //计算公式为： 计数器时钟频率 (fCK_CNT) 等于fCK_PSC / (PSC[15:0] + 1)，可实现 1 至 65536 分频。
    TIM_TimeBaseInitStructure.TIM_Prescaler = psc;

    //时钟分频，设置定时器时钟 CK_INT 频率与死区发生器以及数字滤波器采样时钟频率分频比。可以选择 1、 2、 4 分频。
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up; //设置计数模式，向上计数模式

    //TIM_TimeBaseInitStructure.TIM_RepetitionCounter =  0x00;      //设置重复计数器的值，0。重复计数器，只有 8 位，只存在于高级定时器。
    TIM_TimeBaseInit( TIM2, &TIM_TimeBaseInitStructure);            //初始化


    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;          //TIM1捕获比较中断
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;//设置抢占优先级
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;       //设置响应优先级
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;          //使能通道
    NVIC_Init(&NVIC_InitStructure);

    TIM_ClearFlag(TIM2, TIM_FLAG_Update);
    TIM_ITConfig( TIM2, TIM_IT_Update, ENABLE ); //使能TIM2更新中断
    TIM_Cmd( TIM2, DISABLE );                    //定时器使能

}
/********************************************************************
   * 函 数 名     : TIM6_Init
   * 函数功能  : 初始化 定时器 TIM6
   * 输    入        : arr:自动重装值，psc:预分频系数
   * 输    出        : 无
   ********************************************************************/
void Tick_TIM_Init( u16 arr, u16 psc)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;

    RCC_APB1PeriphClockCmd( RCC_APB1Periph_TIM6, ENABLE );

    TIM_TimeBaseInitStructure.TIM_Period = arr;
    TIM_TimeBaseInitStructure.TIM_Prescaler = psc;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Down;
    TIM_TimeBaseInit( TIM6, &TIM_TimeBaseInitStructure);

    TIM_ITConfig(TIM6, TIM_IT_Update, ENABLE);
    TIM_ARRPreloadConfig( TIM6, ENABLE );
    TIM_Cmd( TIM6, ENABLE );
}

/********************************************************************
   * 函 数 名     : TIM7_Init
   * 函数功能  : 初始化 定时器 TIM7
   * 输    入        : arr:自动重装值，psc:预分频系数
   * 输    出        : 无
   ********************************************************************/
void Refresh_TIM_Init( u16 arr, u16 psc)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;

    RCC_APB1PeriphClockCmd( RCC_APB1Periph_TIM7, ENABLE );

    TIM_TimeBaseInitStructure.TIM_Period = arr;
    TIM_TimeBaseInitStructure.TIM_Prescaler = psc;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Down;
    TIM_TimeBaseInit( TIM7, &TIM_TimeBaseInitStructure);

    TIM_ITConfig(TIM7, TIM_IT_Update, ENABLE);
    TIM_ARRPreloadConfig( TIM7, ENABLE );
    TIM_Cmd( TIM7, ENABLE );
}

/********************************************************************
   * 函 数 名       : TIM6_INT_Init
   * 函数功能    : 初始化定时器中断
   * 输    入          : 无
   * 输    出          : 无
   ********************************************************************/
void Tick_TIM_INT_Init(void)
{
    NVIC_InitTypeDef NVIC_InitStructure={0};
    NVIC_InitStructure.NVIC_IRQChannel = TIM6_IRQn ;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; //抢占优先级
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;        //子优先级
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/********************************************************************
   * 函 数 名       : TIM7_INT_Init
   * 函数功能    : 初始化定时器中断
   * 输    入          : 无
   * 输    出          : 无
   ********************************************************************/
void Refresh_TIM_INT_Init(void)
{
    NVIC_InitTypeDef NVIC_InitStructure={0};
    NVIC_InitStructure.NVIC_IRQChannel = TIM7_IRQn ;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; //抢占优先级
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;        //子优先级
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}
void ENABLE_TIM(void)

{
   // while(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_1)==RESET)
    {
        TIM_SetCounter(TIM2,0);
       // count = 0;
        TIM_Cmd(TIM2,ENABLE);//回响信号到来，开启定时器计数
    }
}

void DISABLE_TIM(void)

{
   // while(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_1)==SET)
    {
        TIM_Cmd(TIM2,DISABLE);//回响信号到来，开启定时器计数
    }
}

float Ultrasoniclength(void )

{
    u32 t = 0;
    int i = 0;
    float length = 0 , sum = 0;

    while(i!=5)
    {
       Start_Trig();
       fengmingqi1_H;
       while(GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_15)==RESET);  //此处一直等，等到为1，进行下一步
       ENABLE_TIM();//回响信号到来，开启定时器计数
       i = i + 1;
       while(GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_15)==SET);  //此处一直等，等到为0，进行下一步，这两段while之间的时间就是高电平时间，即发出到返回接收的时间
       DISABLE_TIM();//关闭定时器计数
       t = TIM_GetCounter(TIM2);
       length=(t+count*1000)/58.0;//通过回响信号计算距离
       sum = length + sum ;
       TIM_SetCounter(TIM2,0);
       count = 0;
       Delay_Ms(100);
     }
     length = sum/5.0;
     distance=length;
     printf("DISTANCE:%3.2f cm\r\n",distance);
     if(distance<10)
     {
         fengmingqi1_L;
     }
     else
     {
         fengmingqi1_H;
     }
     return length;

}



void TIM2_IRQHandler(void)   __attribute__((interrupt("WCH-Interrupt-fast")));
void TIM2_IRQHandler(void)
{
    if(TIM_GetITStatus(TIM2,TIM_IT_Update)!=RESET)
    {
        TIM_ClearITPendingBit(TIM2,TIM_IT_Update);//清除中断标志
        count++;
    }
}
extern uint8_t tick;
extern uint32_t time;
extern uint8_t refresh_flag;

void TIM6_IRQHandler(void)   __attribute__((interrupt("WCH-Interrupt-fast")));
void TIM6_IRQHandler(void)
{
    TIM_ClearFlag(TIM6, TIM_FLAG_Update);//清除标志位
    tick = 1;
    time += 1;
}

void TIM7_IRQHandler(void)   __attribute__((interrupt("WCH-Interrupt-fast")));
void TIM7_IRQHandler(void)
{
    TIM_ClearFlag(TIM7, TIM_FLAG_Update);//清除标志位
    refresh_flag = 1;
}
