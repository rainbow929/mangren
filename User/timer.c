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

    //ʹ��IM2ʱ��
    RCC_APB1PeriphClockCmd( RCC_APB1Periph_TIM2, ENABLE);

    Button_INT_Init();
    TIM_DeInit(TIM2);

    //��ʱ�����ڣ�ʵ�ʾ����趨�Զ����ؼĴ��� ARR ��ֵ�� ARR ΪҪװ�ص�ʵ���Զ����ؼĴ�������Ӱ�ӼĴ����� ��ֵ�� �����÷�ΧΪ 0 �� 65535��
    TIM_TimeBaseInitStructure.TIM_Period = arr;

    //��ʱ��Ԥ��Ƶ�����ã�ʱ��Դ����Ԥ��Ƶ�����Ƕ�ʱ������ʱ��CK_CNT�����趨 PSC �Ĵ�����ֵ��
    //���㹫ʽΪ�� ������ʱ��Ƶ�� (fCK_CNT) ����fCK_PSC / (PSC[15:0] + 1)����ʵ�� 1 �� 65536 ��Ƶ��
    TIM_TimeBaseInitStructure.TIM_Prescaler = psc;

    //ʱ�ӷ�Ƶ�����ö�ʱ��ʱ�� CK_INT Ƶ���������������Լ������˲�������ʱ��Ƶ�ʷ�Ƶ�ȡ�����ѡ�� 1�� 2�� 4 ��Ƶ��
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up; //���ü���ģʽ�����ϼ���ģʽ

    //TIM_TimeBaseInitStructure.TIM_RepetitionCounter =  0x00;      //�����ظ���������ֵ��0���ظ���������ֻ�� 8 λ��ֻ�����ڸ߼���ʱ����
    TIM_TimeBaseInit( TIM2, &TIM_TimeBaseInitStructure);            //��ʼ��


    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;          //TIM1����Ƚ��ж�
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;//������ռ���ȼ�
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;       //������Ӧ���ȼ�
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;          //ʹ��ͨ��
    NVIC_Init(&NVIC_InitStructure);

    TIM_ClearFlag(TIM2, TIM_FLAG_Update);
    TIM_ITConfig( TIM2, TIM_IT_Update, ENABLE ); //ʹ��TIM2�����ж�
    TIM_Cmd( TIM2, DISABLE );                    //��ʱ��ʹ��

}
/********************************************************************
   * �� �� ��     : TIM6_Init
   * ��������  : ��ʼ�� ��ʱ�� TIM6
   * ��    ��        : arr:�Զ���װֵ��psc:Ԥ��Ƶϵ��
   * ��    ��        : ��
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
   * �� �� ��     : TIM7_Init
   * ��������  : ��ʼ�� ��ʱ�� TIM7
   * ��    ��        : arr:�Զ���װֵ��psc:Ԥ��Ƶϵ��
   * ��    ��        : ��
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
   * �� �� ��       : TIM6_INT_Init
   * ��������    : ��ʼ����ʱ���ж�
   * ��    ��          : ��
   * ��    ��          : ��
   ********************************************************************/
void Tick_TIM_INT_Init(void)
{
    NVIC_InitTypeDef NVIC_InitStructure={0};
    NVIC_InitStructure.NVIC_IRQChannel = TIM6_IRQn ;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; //��ռ���ȼ�
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;        //�����ȼ�
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/********************************************************************
   * �� �� ��       : TIM7_INT_Init
   * ��������    : ��ʼ����ʱ���ж�
   * ��    ��          : ��
   * ��    ��          : ��
   ********************************************************************/
void Refresh_TIM_INT_Init(void)
{
    NVIC_InitTypeDef NVIC_InitStructure={0};
    NVIC_InitStructure.NVIC_IRQChannel = TIM7_IRQn ;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; //��ռ���ȼ�
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;        //�����ȼ�
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}
void ENABLE_TIM(void)

{
   // while(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_1)==RESET)
    {
        TIM_SetCounter(TIM2,0);
       // count = 0;
        TIM_Cmd(TIM2,ENABLE);//�����źŵ�����������ʱ������
    }
}

void DISABLE_TIM(void)

{
   // while(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_1)==SET)
    {
        TIM_Cmd(TIM2,DISABLE);//�����źŵ�����������ʱ������
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
       while(GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_15)==RESET);  //�˴�һֱ�ȣ��ȵ�Ϊ1��������һ��
       ENABLE_TIM();//�����źŵ�����������ʱ������
       i = i + 1;
       while(GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_15)==SET);  //�˴�һֱ�ȣ��ȵ�Ϊ0��������һ����������while֮���ʱ����Ǹߵ�ƽʱ�䣬�����������ؽ��յ�ʱ��
       DISABLE_TIM();//�رն�ʱ������
       t = TIM_GetCounter(TIM2);
       length=(t+count*1000)/58.0;//ͨ�������źż������
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
        TIM_ClearITPendingBit(TIM2,TIM_IT_Update);//����жϱ�־
        count++;
    }
}
extern uint8_t tick;
extern uint32_t time;
extern uint8_t refresh_flag;

void TIM6_IRQHandler(void)   __attribute__((interrupt("WCH-Interrupt-fast")));
void TIM6_IRQHandler(void)
{
    TIM_ClearFlag(TIM6, TIM_FLAG_Update);//�����־λ
    tick = 1;
    time += 1;
}

void TIM7_IRQHandler(void)   __attribute__((interrupt("WCH-Interrupt-fast")));
void TIM7_IRQHandler(void)
{
    TIM_ClearFlag(TIM7, TIM_FLAG_Update);//�����־λ
    refresh_flag = 1;
}
