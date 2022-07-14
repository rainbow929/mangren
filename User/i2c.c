/*
 * i2c.c
 *
 *  Created on: Mar 22, 2022
 *      Author: ExplodingONC
 */

#include "i2c.h"
#include "debug.h"

/*******************************************************************************
 * Function Name  : IIC_Init
 * Description    : Initializes the IIC peripheral.
 * Input          : None
 * Return         : None
 *******************************************************************************/
void IIC_Init(u32 bound, u16 address)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  I2C_InitTypeDef I2C_InitTSturcture;

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
  //  GPIO_PinRemapConfig(GPIO_Remap_I2C1, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  I2C_InitTSturcture.I2C_ClockSpeed = bound;
  I2C_InitTSturcture.I2C_Mode = I2C_Mode_I2C;
  I2C_InitTSturcture.I2C_DutyCycle = I2C_DutyCycle_16_9;
  I2C_InitTSturcture.I2C_OwnAddress1 = address;
  I2C_InitTSturcture.I2C_Ack = I2C_Ack_Enable;
  I2C_InitTSturcture.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  I2C_Init(I2C2, &I2C_InitTSturcture);

  I2C_Cmd(I2C2, ENABLE);

  I2C_AcknowledgeConfig(I2C2, ENABLE);
}
