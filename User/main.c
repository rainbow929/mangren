/********************************** (C) COPYRIGHT *******************************
* File Name          : main.c
* Author             : ExplodingONC
* Version            : V1.0.0
* Date               : 2022/03/24
* Description        : ˤ������
* Copyright
* SPDX-License-Identifier: Apache-2.0
*******************************************************************************/



#include "math.h"
#include "debug.h"
#include "timer.h"
#include "gpio.h"
#include "imu_mpu6050.h"
#include "lcd_st7789.h"
#include "images.h"
#include "ov.h"
#include "ch32v30x_conf.h"

/* Global typedef */

/* Global define */
#define TRUE 1
#define FALSE 0

#define RXBUF_SIZE 1024     // DMA buffer size
#define size(a)   (sizeof(a) / sizeof(*(a)))

/* Global Variable */
u8 TxBuffer[] = " ";
u8 RxBuffer[RXBUF_SIZE]={0};

uint32_t sqrt32(uint32_t n);

/* Global Variable */
uint8_t tick = 0;
uint32_t time, time_stamp_trigger, time_stamp_record;
uint8_t refresh_flag = 0;

int16_t raw_acc[3], raw_gyro[3];
float acc[3], gyro[3];
int32_t raw_acc_total, raw_gyro_total;
int32_t acc_total_100x, gyro_total_10x;
int32_t acc_record_100x;
int32_t acc_threshold_warning_100x = 2500;
int32_t acc_threshold_crack_100x = 10000;
uint8_t warning_flag = 0, crack_flag = 0, crack_halt_flag = 0, crack_repair_flag = 0;
extern float distance;

/*******************************************************************************
* Function Name  : USARTx_CFG
* Description    : Initializes the USART peripheral.
* ����    ��   ���ڳ�ʼ��
* Input          : None
* Return         : None
*******************************************************************************/
void USARTx_CFG(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    //����ʱ��
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART7, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

    /* USART7 TX-->C2  RX-->C3 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;           //RX����������
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    USART_InitStructure.USART_BaudRate = 115200;                    // ������
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;     // ����λ 8
    USART_InitStructure.USART_StopBits = USART_StopBits_1;          // ֹͣλ 1
    USART_InitStructure.USART_Parity = USART_Parity_No;             // ��У��
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // ��Ӳ������
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx; //ʹ�� RX �� TX

    USART_Init(UART7, &USART_InitStructure);
    DMA_Cmd(DMA2_Channel9, ENABLE);                                  //�������� DMA
    USART_Cmd(UART7, ENABLE);                                        //����UART
}

/*******************************************************************************
* Function Name  : DMA_INIT
* Description    : Configures the DMA.
* ����    ��   DMA ��ʼ��
* Input          : None
* Return         : None
*******************************************************************************/
void DMA_INIT(void)
{
    DMA_InitTypeDef DMA_InitStructure;
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2, ENABLE);

    // TX DMA ��ʼ��
    DMA_DeInit(DMA2_Channel8);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)(&UART7->DATAR);        // DMA �����ַ����ָ���Ӧ������
    DMA_InitStructure.DMA_MemoryBaseAddr = (u32)TxBuffer;                   // DMA �ڴ��ַ��ָ���ͻ��������׵�ַ
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;                      // ���� : ���� ��Ϊ �յ㣬�� �ڴ� ->  ����
    DMA_InitStructure.DMA_BufferSize = 0;                                   // ��������С,��ҪDMA���͵����ݳ���,Ŀǰû�����ݿɷ�
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;        // �����ַ����������
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                 // �ڴ��ַ����������
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; // ��������λ��8λ(Byte)
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;         // �ڴ�����λ��8λ(Byte)
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                           // ��ͨģʽ�������������ѭ������
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;                 // ���ȼ����
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;                            // M2P,����M2M
    DMA_Init(DMA2_Channel8, &DMA_InitStructure);

    // RX DMA ��ʼ�������λ������Զ�����
    DMA_DeInit(DMA2_Channel9);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)(&UART7->DATAR);
    DMA_InitStructure.DMA_MemoryBaseAddr = (u32)RxBuffer;                   // ���ջ�����
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;                      // ���� : ���� ��Ϊ Դ���� �ڴ� <- ����
    DMA_InitStructure.DMA_BufferSize = RXBUF_SIZE;                          // ����������Ϊ RXBUF_SIZE
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;                         // ѭ��ģʽ�����ɻ��λ�����
    DMA_Init(DMA2_Channel9, &DMA_InitStructure);

}


/*******************************************************************************
* Function Name  : GPIO_CFG
* Description    : Initializes GPIOs.
* ����    ��   GPIO����ʼ��
* Input          : None
* Return         : None
*******************************************************************************/
void GPIO_CFG(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    // CH9141 �������ų�ʼ��
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    /* BLE_sleep --> C13  BLE_AT-->A7 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
}
/*******************************************************************************
* Function Name  :  uartWriteBLE
* Description    :  send data to BLE via UART7          ������ģ�鷢������
* Input          :  char * data          data to send   Ҫ���͵����ݵ��׵�ַ
*                   uint16_t num         number of data ���ݳ���
* Return         :  RESET                UART7 busy,failed to send  ����ʧ��
*                   SET                  send success               ���ͳɹ�
*******************************************************************************/
FlagStatus uartWriteBLE(char * data , uint16_t num)
{
    //���ϴη���δ��ɣ�����
    if(DMA_GetCurrDataCounter(DMA2_Channel8) != 0){
        return RESET;
    }

    DMA_ClearFlag(DMA2_FLAG_TC8);
    DMA_Cmd(DMA2_Channel8, DISABLE );           // �� DMA �����
    DMA2_Channel8->MADDR = (uint32_t)data;      // ���ͻ�����Ϊ data
    DMA_SetCurrDataCounter(DMA2_Channel8,num);  // ���û���������
    DMA_Cmd(DMA2_Channel8, ENABLE);             // �� DMA
    return SET;
}

/*******************************************************************************
* Function Name  :  uartWriteBLEstr
* Description    :  send string to BLE via UART7    ������ģ�鷢���ַ���
* Input          :  char * str          string to send
* Return         :  RESET                UART7 busy,failed to send  ����ʧ��
*                   SET                  send success               ���ͳɹ�
*******************************************************************************/
FlagStatus uartWriteBLEstr(char * str)
{
    uint16_t num = 0;
    while(str[num])num++;           // �����ַ�������
    return uartWriteBLE(str,num);
}


/*******************************************************************************
* Function Name  :  uartReadBLE
* Description    :  read some bytes from receive buffer �ӽ��ջ���������һ������
* Input          :  char * buffer        buffer to storage the data ������Ŷ������ݵĵ�ַ
*                   uint16_t num         number of data to read     Ҫ�����ֽ���
* Return         :  int                  number of bytes read       ����ʵ�ʶ������ֽ���
*******************************************************************************/
uint16_t rxBufferReadPos = 0;       //���ջ�������ָ��
uint32_t uartReadBLE(char * buffer , uint16_t num)
{
    uint16_t rxBufferEnd = RXBUF_SIZE - DMA_GetCurrDataCounter(DMA2_Channel9); //���� DMA ����β��λ��
    uint16_t i = 0;

    if (rxBufferReadPos == rxBufferEnd){
        // �����ݣ�����
        return 0;
    }

    while (rxBufferReadPos!=rxBufferEnd && i < num){
        buffer[i] = RxBuffer[rxBufferReadPos];
        i++;
        rxBufferReadPos++;
        if(rxBufferReadPos >= RXBUF_SIZE){
            // ����������������
            rxBufferReadPos = 0;
        }
    }
    return i;
}

/*******************************************************************************
* Function Name  :  uartReadByteBLE
* Description    :  read one byte from UART buffer  �ӽ��ջ��������� 1 �ֽ�����
* Input          :  None
* Return         :  char    read data               ���ض���������(������Ҳ����0)
*******************************************************************************/
char uartReadByteBLE()
{
    char ret;
    uint16_t rxBufferEnd = RXBUF_SIZE - DMA_GetCurrDataCounter(DMA2_Channel9);//���� DMA ����β��λ��
    if (rxBufferReadPos == rxBufferEnd){
        // �����ݣ�����
        return 0;
    }
    ret = RxBuffer[rxBufferReadPos];
    rxBufferReadPos++;
    if(rxBufferReadPos >= RXBUF_SIZE)
    {
        // ����������������
        rxBufferReadPos = 0;
    }
    return ret;
}
/*******************************************************************************
* Function Name  :  uartAvailableBLE
* Description    :  get number of bytes Available to read from the UART buffer  ��ȡ�������пɶ����ݵ�����
* Input          :  None
* Return         :  uint16_t    number of bytes Available to readd              ���ؿɶ���������
*******************************************************************************/
uint16_t uartAvailableBLE()
{
    uint16_t rxBufferEnd = RXBUF_SIZE - DMA_GetCurrDataCounter(DMA2_Channel9);//���� DMA ����β��λ��
    // ����ɶ��ֽ�
    if (rxBufferReadPos <= rxBufferEnd){
        return rxBufferEnd - rxBufferReadPos;
    }else{
        return rxBufferEnd +RXBUF_SIZE -rxBufferReadPos;
    }
}
void USART1_Init(void)
   {
      GPIO_InitTypeDef  GPIO_InitStructure;
      USART_InitTypeDef USART_InitStructure;
      //����ʱ��
      RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);

      GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
      GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
      GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
      GPIO_Init(GPIOA, &GPIO_InitStructure);


      USART_InitStructure.USART_BaudRate = 115200;                    // ������
      USART_InitStructure.USART_WordLength = USART_WordLength_8b;
      USART_InitStructure.USART_StopBits = USART_StopBits_1;
      USART_InitStructure.USART_Parity = USART_Parity_No;
      USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
      USART_InitStructure.USART_Mode = USART_Mode_Tx;

      USART_Init(USART1, &USART_InitStructure);

      USART_Cmd(USART1, ENABLE);                                     //����UART
   }

void EXTI_INT_INIT(void)
{
   GPIO_InitTypeDef  GPIO_InitStructure={0};
   EXTI_InitTypeDef EXTI_InitStructure={0};
   NVIC_InitTypeDef NVIC_InitStructure={0};

   RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO|RCC_APB2Periph_GPIOA,ENABLE);

   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
   GPIO_Init(GPIOA, &GPIO_InitStructure);

   /* GPIOA ----> EXTI_Line0 */
   GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource0);
   EXTI_InitStructure.EXTI_Line=EXTI_Line0;
   EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
   EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising; //����Ϊ�ߵ�ƽ����������
   EXTI_InitStructure.EXTI_LineCmd = ENABLE;
   EXTI_Init(&EXTI_InitStructure);

   NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn ;
   NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; //��ռ���ȼ�
   NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;        //�����ȼ�
   NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
   NVIC_Init(&NVIC_InitStructure);
}


/* DVP Work Mode */
#define RGB565_MODE   0
/* DVP Work Mode Selection */
#define DVP_Work_Mode    RGB565_MODE

UINT32  JPEG_DVPDMAaddr0 = 0x20005000;
UINT32  JPEG_DVPDMAaddr1 = 0x20005000 + OV2640_JPEG_WIDTH;

UINT32  RGB565_DVPDMAaddr0 = 0x2000A000;
UINT32  RGB565_DVPDMAaddr1 = 0x2000A000 + RGB565_COL_NUM*2;


volatile UINT32 frame_cnt = 0;
volatile UINT32 addr_cnt = 0;
volatile UINT32 href_cnt = 0;

void DVP_IRQHandler (void) __attribute__((interrupt("WCH-Interrupt-fast")));


/*******************************************************************************
* Function Name  : LCD_Reset_GPIO_Init
* Description    : Init LCD reset GPIO.
* Input          : None
* Return         : None
*******************************************************************************/
void LCD_Reset_GPIO_Init(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    GPIO_SetBits(GPIOD,GPIO_Pin_13);
}

/*******************************************************************************
* Function Name  : DMA_SRAMLCD_Init
* Description    : Init SRAMLCD DMA
* Input          : ddr: DVP data memory base addr.
* Return         : None
*******************************************************************************/
void DMA_SRAMLCD_Init(u32 ddr)
{
    DMA_InitTypeDef DMA_InitStructure;

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2, ENABLE);

    DMA_DeInit(DMA2_Channel5);

    DMA_InitStructure.DMA_PeripheralBaseAddr = ddr;
    DMA_InitStructure.DMA_MemoryBaseAddr = (u32)LCD_DATA;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_BufferSize = 0;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Enable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Enable;
    DMA_Init(DMA2_Channel5, &DMA_InitStructure);
}

/*******************************************************************************
* Function Name  : DMA_SRAMLCD_Enable
* Description    : Enable SRAMLCD DMA transmission
* Input          : None
* Return         : None
*******************************************************************************/
void DMA_SRAMLCD_Enable(void)
{
    DMA_Cmd(DMA2_Channel5, DISABLE );
    DMA_SetCurrDataCounter(DMA2_Channel5,LCD_W);
    DMA_Cmd(DMA2_Channel5, ENABLE);
}

/*******************************************************************************
* Function Name  : DVP_Init
* Description    : Init DVP
* Input          : None
* Return         : None
*******************************************************************************/
void DVP_Init(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DVP, ENABLE);

    DVP->CR0 &= ~RB_DVP_MSK_DAT_MOD;

#if (DVP_Work_Mode == RGB565_MODE)
    /* VSYNC��HSYNC:High level active */
    DVP->CR0 |= RB_DVP_D10_MOD | RB_DVP_V_POLAR;
    DVP->CR1 &= ~((RB_DVP_ALL_CLR)| RB_DVP_RCV_CLR);
    DVP->ROW_NUM = RGB565_ROW_NUM;               // rows
    DVP->COL_NUM = RGB565_COL_NUM*2;               // cols

    DVP->DMA_BUF0 = RGB565_DVPDMAaddr0;      //DMA addr0
    DVP->DMA_BUF1 = RGB565_DVPDMAaddr1;      //DMA addr1

#endif

    /* Set frame capture rate */
    DVP->CR1 &= ~RB_DVP_FCRC;
    DVP->CR1 |= DVP_RATE_100P;  //100%

    //Interupt Enable
    DVP->IER |= RB_DVP_IE_STP_FRM;
    DVP->IER |= RB_DVP_IE_FIFO_OV;
    DVP->IER |= RB_DVP_IE_FRM_DONE;
    DVP->IER |= RB_DVP_IE_ROW_DONE;
    DVP->IER |= RB_DVP_IE_STR_FRM;

    NVIC_InitStructure.NVIC_IRQChannel = DVP_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    DVP->CR1 |= RB_DVP_DMA_EN;  //enable DMA
    DVP->CR0 |= RB_DVP_ENABLE;  //enable DVP
}

u32 DVP_ROW_cnt=0;

/*******************************************************************************
* Function Name  : DVP_IRQHandler
* Description    : This function handles DVP exception.
* Input          : None
* Return         : None
*******************************************************************************/
void DVP_IRQHandler(void)
{

    if (DVP->IFR & RB_DVP_IF_ROW_DONE)
    {
        /* Write 0 clear 0 */
        DVP->IFR &= ~RB_DVP_IF_ROW_DONE;  //clear Interrupt

#if (DVP_Work_Mode == RGB565_MODE)
        if (addr_cnt%2)     //buf1 done
        {
            addr_cnt++;
            //Send DVP data to LCD

            DMA_Cmd(DMA2_Channel5, DISABLE );

            for (u16 i = 0;i < RGB565_COL_NUM; ++i) {
                *(u8 *)(RGB565_DVPDMAaddr0+i)=(u8)(*(u16 *)(RGB565_DVPDMAaddr0+i*2)>>2);
            }
            DMA_SetCurrDataCounter(DMA2_Channel5,RGB565_COL_NUM);
            DMA2_Channel5->PADDR = RGB565_DVPDMAaddr0;
            DMA_Cmd(DMA2_Channel5, ENABLE);

        }
        else                //buf0 done
        {
            addr_cnt++;
            //Send DVP data to LCD

            DMA_Cmd(DMA2_Channel5, DISABLE );

            for (u16 i = 0;i < RGB565_COL_NUM; ++i) {
                *(u8 *)(RGB565_DVPDMAaddr1+i)=(u8)(*(u16 *)(RGB565_DVPDMAaddr1+i*2)>>2);
            }
            DMA_SetCurrDataCounter(DMA2_Channel5,RGB565_COL_NUM);
            DMA2_Channel5->PADDR = RGB565_DVPDMAaddr1;
            DMA_Cmd(DMA2_Channel5, ENABLE);
        }

        href_cnt++;

#endif

    }

    if (DVP->IFR & RB_DVP_IF_FRM_DONE)
    {
        DVP->IFR &= ~RB_DVP_IF_FRM_DONE;  //clear Interrupt

#if (DVP_Work_Mode == RGB565_MODE)

        addr_cnt = 0;
        href_cnt = 0;

#endif

    }

    if (DVP->IFR & RB_DVP_IF_STR_FRM)
    {
        DVP->IFR &= ~RB_DVP_IF_STR_FRM;

        frame_cnt++;
    }

    if (DVP->IFR & RB_DVP_IF_STP_FRM)
    {
        DVP->IFR &= ~RB_DVP_IF_STP_FRM;

    }

    if (DVP->IFR & RB_DVP_IF_FIFO_OV)
    {
        DVP->IFR &= ~RB_DVP_IF_FIFO_OV;

        printf("FIFO OV\r\n");
    }

}
void shexiang(void)
{
    while(OV2640_Init())
     {
       LCD_SetColor(0x18E3, WHITE);
       LCD_ShowString(10, 50, 24, TRUE, "Camera Err");
       Delay_Ms(500);
       LCD_SetColor(0x18E3, WHITE);
       LCD_ShowString(10, 50, 24, TRUE, "          ");
     }

     LCD_AddressSetWrite(0,120,239,239);//0 80 239 239
     Delay_Ms(100);
     RGB565_Mode_Init();
     Delay_Ms(100);

     #if (DVP_Work_Mode == RGB565_MODE)

     #endif
     DMA_SRAMLCD_Init((u32)RGB565_DVPDMAaddr0);  //DMA2
     DVP_Init();
}

void shuaidao(void)
{
    if (tick)
    {
        if (!crack_flag)    // pause data collection if crack happens
        {               // data collection
            IMU_GetAccelerometer(&acc[0], &acc[1], &acc[2]);
            IMU_GetGyroscope(&gyro[0], &gyro[1], &gyro[2]);
            acc_total_100x = sqrt32( (int)( 10000* ( acc[0]*acc[0] + acc[1]*acc[1] + acc[2]*acc[2] ) ) );
            gyro_total_10x = sqrt32( (int)( 100* ( gyro[0]*gyro[0] + gyro[1]*gyro[1] + gyro[2]*gyro[2] ) ) );                // IMU warning
            if (acc_total_100x > acc_threshold_warning_100x)
            {
                time_stamp_trigger = time; warning_flag = 1;
                if ( (acc_total_100x > acc_record_100x) || (time - time_stamp_record > 1000) )
                {
                    time_stamp_record = time;
                    acc_record_100x = acc_total_100x;
                }
            }
            else if (time - time_stamp_trigger >= 1000)
            {
                warning_flag = 0;
                acc_record_100x = 0;
            }                // IMU glass crack
            if ( ( acc_total_100x > acc_threshold_crack_100x ) )
            {
                if ( abs(acc[0]) > abs(acc[1]) )
                {
                    if ( acc[0] > 0 ) {    // east
                        crack_flag = 0x10;
                        if ( acc[1] > 0 ) crack_flag |= 0x02;  // east-north
                        else crack_flag |= 0x08;                // east-south
                    }
                    else {                  // west
                        crack_flag = 0x40;
                        if ( acc[1] > 0 ) crack_flag |= 0x02;  // west-north
                        else crack_flag |= 0x08;                // west-south
                    }
                }
                else
                {
                    if ( acc[1] > 0 ) {    // north
                        crack_flag = 0x20;
                        if ( acc[0] > 0 ) crack_flag |= 0x01;  // north-east
                        else crack_flag |= 0x04;                // north-west
                    }
                    else {                  // south
                        crack_flag = 0x80;
                        if ( acc[0] > 0 ) crack_flag |= 0x01;  // south-east
                        else crack_flag |= 0x04;                // south-west
                    }
                }
            }
        }
        tick = 0;
    }
    if (refresh_flag && !crack_halt_flag)
    {
        if (crack_repair_flag)
        {

            LCD_SetColor(0x18E3, GRAY);//WHITE
            crack_repair_flag = 0;
        }
        if (warning_flag)
            {
             LCD_SetColor(0x18E3, BLUE);//RED
             LCD_ShowString(10, 40, 16, TRUE, "State:WAVE");
             LCD_SetColor(0x18E3, WHITE);
             LCD_ShowString(10, 80, 16, TRUE, "Pedestrian traffic is normal");
            }
        else
           {
            LCD_SetColor(0x18E3, WHITE);//GRAY
            LCD_ShowString(10, 40, 16, TRUE, "State:REST");
            LCD_ShowString(10, 80, 16, TRUE, "Pedestrian traffic is normal");
           }
        if (crack_flag)
        {
            LCD_SetColor(0x18E3, RED);
            LCD_ShowString(10, 40, 16, TRUE, "State:HELP");
            LCD_ShowString(0, 80, 16, TRUE, "Pedestrian traffic is abnormal");
            LCD_SetColor(BLACK, RED);
            shexiang();
        }
        refresh_flag = 0;
    }
}
/*********************************************************************
 * @fn      main
 *
 * @brief   Main program.
 *
 * @return  none
 */
int main(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	Delay_Init();
	USART_Printf_Init(115200);

	fengmingqi1_H;
	// report clock rate
	u32 SystemCoreClock_MHz = SystemCoreClock / 1000000;
	u32 SystemCoreClock_kHz = (SystemCoreClock % 1000000) / 1000;
	//printf("System Clock Speed = %d.%03dMHz\r\n", SystemCoreClock_MHz,SystemCoreClock_kHz);

	LCD_Init();
	Delay_Ms(100);
	LCD_SetBrightness(75);
	crack_repair_flag = 1;

	DMA_INIT();
    USARTx_CFG();                                                 /* USART INIT */
    USART_DMACmd(UART7,USART_DMAReq_Tx|USART_DMAReq_Rx,ENABLE);

    GPIO_CFG();
    GPIO_WriteBit(GPIOA, GPIO_Pin_7,RESET); //���� AT
    GPIO_WriteBit(GPIOC, GPIO_Pin_13,SET); //enable CH9141
    Delay_Ms(1000);

	LCD_SetColor(0x18E3, WHITE);
	LCD_ShowString(10, 0, 24, TRUE, "Blind Navigation");

	LCD_SetColor(0x18E3, WHITE);
	LCD_ShowString(10, 100, 16, TRUE, "Street Conditions");

	LCD_SetColor(0x18E3, WHITE);
	LCD_ShowString(10, 60, 16, TRUE, "Distance:");

    IMU_Init();
    IMU_SetGyroFsr(0x02);
    IMU_SetAccelFsr(0x03);
    IMU_SetRate(1000);
    IMU_SetLPF(500);

    tick = 1; time = 0; refresh_flag = 1;
    Button_INT_Init();  // wakeup button
    Tick_TIM_Init( 1000-1, SystemCoreClock_MHz-1 ); Tick_TIM_INT_Init();        // 1kHz tick & time
    Refresh_TIM_Init( 33333-1, SystemCoreClock_MHz-1 ); Refresh_TIM_INT_Init(); // 30Hz refresh_flag


    Input_Capture_Init(1000-1,72-1);

	while(1)
    {
	    Delay_Us(10);	    // executed every 1ms
        LCD_SetColor(0x18E3, BLACK);//RED


        GPIO_WriteBit(GPIOA, GPIO_Pin_7,SET); // �˳�AT�������ֻ����������CH9141,���������շ�
        EXTI_INT_INIT();

        //shuaidao();
        shexiang();

        Ultrasoniclength();  //cm
        Delay_Ms(100);




	}
}


//square root function for int32
uint32_t sqrt32(uint32_t n)
{
    uint32_t c = 0x8000;
    uint32_t g = 0x8000;
    for(;;)
    {
        if(g*g > n)
            g ^= c;
        c >>= 1;
        if(c == 0)
            return g;
        g |= c;
    }
    return 0;
}


