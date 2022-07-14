/*
 * imu_mpu6050.c
 *
 *  Created on: Mar 22, 2022
 *      Author: ExplodingONC
 */

#include "imu_mpu6050.h"

uint8_t gyro_fsr = 0;
uint8_t accel_fsr = 0;


//��ʼ��MPU6050
//����ֵ: 0,�ɹ�
//       ����,�������
u8 IMU_Init(void)
{
    IIC_Init(100000, 0x02);                 //��ʼ��IIC����
    IMU_WriteByte(MPU_PWR_MGMT1_REG, 0X80); //��λMPU6050
    Delay_Ms(500);
    IMU_WriteByte(MPU_PWR_MGMT1_REG, 0X00); //����MPU6050
    IMU_SetGyroFsr(0x03);
    IMU_SetAccelFsr(0x03);
    IMU_SetRate(1000);
    IMU_SetLPF(500);
    IMU_WriteByte(MPU_INT_EN_REG, 0X00);    // �ر������ж�
    IMU_WriteByte(MPU_USER_CTRL_REG, 0X00); // I2C��ģʽ�ر�
    IMU_WriteByte(MPU_FIFO_EN_REG, 0X00);   // �ر�FIFO
    IMU_WriteByte(MPU_INTBP_CFG_REG, 0X80); // INT���ŵ͵�ƽ��Ч
    if (IMU_ReadByte(MPU_DEVICE_ID_REG) == MPU_ADDR) //����ID��ȷ
    {
        IMU_WriteByte(MPU_PWR_MGMT1_REG, 0X01); //����CLKSEL,PLL X��Ϊ�ο�
        IMU_WriteByte(MPU_PWR_MGMT2_REG, 0X00); //���ٶ��������Ƕ�����
    }
    else
        return 1;
    return 0;
}

//����MPU6050�����Ǵ����������̷�Χ
//fsr:0,��250dps;1,��500dps;2,��1000dps;3,��2000dps
//����ֵ:0,���óɹ�
//    ����,����ʧ��
u8 IMU_SetGyroFsr(u8 fsr)
{
    gyro_fsr = fsr;
    return IMU_WriteByte(MPU_GYRO_CFG_REG, fsr << 3); //���������������̷�Χ
}

//����MPU6050���ٶȴ����������̷�Χ
//fsr:0,��2g;1,��4g;2,��8g;3,��16g
//����ֵ:0,���óɹ�
//    ����,����ʧ��
u8 IMU_SetAccelFsr(u8 fsr)
{
    accel_fsr = fsr;
    return IMU_WriteByte(MPU_ACCEL_CFG_REG, fsr << 3); //���ü��ٶȴ����������̷�Χ
}

//����MPU6050�����ֵ�ͨ�˲���
//lpf:���ֵ�ͨ�˲�Ƶ��(Hz)
//����ֵ:0,���óɹ�
//    ����,����ʧ��
u8 IMU_SetLPF(u16 lpf)
{
    u8 data = 0;
    if (lpf >= 188)
        data = 1;
    else if (lpf >= 98)
        data = 2;
    else if (lpf >= 42)
        data = 3;
    else if (lpf >= 20)
        data = 4;
    else if (lpf >= 10)
        data = 5;
    else
        data = 6;
    return IMU_WriteByte(MPU_CFG_REG, data); //�������ֵ�ͨ�˲���
}

//����MPU6050�Ĳ�����(�ٶ�Fs=1KHz)
//rate:4~1000(Hz)
//����ֵ:0,���óɹ�
//    ����,����ʧ��
u8 IMU_SetRate(u16 rate)
{
    u8 data;
    if (rate > 1000)
        rate = 1000;
    if (rate < 4)
        rate = 4;
    data = 1000 / rate - 1;
    return IMU_WriteByte(MPU_SAMPLE_RATE_REG, data); //�������ֵ�ͨ�˲���
}

//�õ��¶�ֵ
//����ֵ:�¶�ֵ(������100��)
short IMU_GetTemperature(void)
{
    u8 buf[2];
    u16 raw;
    float temp;
    IMU_ReadLen(MPU_ADDR, MPU_TEMP_OUTH_REG, 2, buf);
    raw = ((u16)buf[0] << 8) | buf[1];
    temp = 36.53 + ((double)raw) / 340;
    return temp * 100;
}

//�õ�������ֵ(ԭʼֵ)
//gx,gy,gz:������x,y,z���ԭʼ����(������)
//����ֵ:0,�ɹ�
//    ����,�������
u8 IMU_GetRawGyroscope(u16 *gx, u16 *gy, u16 *gz)
{
    u8 buf[6], res;
    res = IMU_ReadLen(MPU_ADDR, MPU_GYRO_XOUTH_REG, 6, buf);
    if (res == 0)
    {
        *gx = ((u16)buf[0] << 8) | buf[1];
        *gy = ((u16)buf[2] << 8) | buf[3];
        *gz = ((u16)buf[4] << 8) | buf[5];
    }
    return res;
}

//�õ�������ֵ(����ֵ)
//gx,gy,gz:������x,y,z��Ķ���(��/s,������)
//����ֵ:0,�ɹ�
//    ����,�������
u8 IMU_GetGyroscope(float *gx, float *gy, float *gz)
{
    u8 buf[6], res;
    res = IMU_ReadLen(MPU_ADDR, MPU_GYRO_XOUTH_REG, 6, buf);
    if (res == 0)
    {
        *gx = IMU_ConvertGyro(((u16)buf[0] << 8) | buf[1]);
        *gy = IMU_ConvertGyro(((u16)buf[2] << 8) | buf[3]);
        *gz = IMU_ConvertGyro(((u16)buf[4] << 8) | buf[5]);
    }
    return res;
}

//�õ����ٶ�ֵ(ԭʼֵ)
// gx,gy,gz:������x,y,z���ԭʼ����(������)
//����ֵ:0,�ɹ�
//    ����,�������
u8 IMU_GetRawAccelerometer(u16 *ax, u16 *ay, u16 *az)
{
    u8 buf[6], res;
    res = IMU_ReadLen(MPU_ADDR, MPU_ACCEL_XOUTH_REG, 6, buf);
    if (res == 0)
    {
        *ax = ((u16)buf[0] << 8) | buf[1];
        *ay = ((u16)buf[2] << 8) | buf[3];
        *az = ((u16)buf[4] << 8) | buf[5];
    }
    return res;
}

//�õ����ٶ�ֵ(����ֵ)
// gx,gy,gz:������x,y,z��Ķ���(m/s2,������)
//����ֵ:0,�ɹ�
//    ����,�������
u8 IMU_GetAccelerometer(float *ax, float *ay, float *az)
{
    u8 buf[6], res;
    res = IMU_ReadLen(MPU_ADDR, MPU_ACCEL_XOUTH_REG, 6, buf);
    if (res == 0)
    {
        *ax = IMU_ConvertAccel(((u16)buf[0] << 8) | buf[1]);
        *ay = IMU_ConvertAccel(((u16)buf[2] << 8) | buf[3]);
        *az = IMU_ConvertAccel(((u16)buf[4] << 8) | buf[5]);
    }
    return res;
}

float IMU_ConvertGyro(s16 raw_gyro)
{
    float gyro;
    // fsr: 0,��250dps; 1,��500dps; 2,��1000dps; 3,��2000dps
    switch(gyro_fsr)
    {
        case 0x00:
            gyro = ((float)raw_gyro) * 250 / 32768;
            break;
        case 0x01:
            gyro = ((float)raw_gyro) * 500 / 32768;
            break;
        case 0x02:
            gyro = ((float)raw_gyro) * 1000 / 32768;
            break;
        case 0x03:
            gyro = ((float)raw_gyro) * 2000 / 32768;
            break;
        default:
            gyro = 0;
    }
    return gyro;
}

float IMU_ConvertAccel(s16 raw_accel)
{
    float accel;
    // fsr: 0,��2g; 1,��4g; 2,��8g; 3,��16g
    switch(accel_fsr)
    {
        case 0x00:
            accel = ((float)raw_accel) * 9.8 *2 / 32768;
            break;
        case 0x01:
            accel = ((float)raw_accel) * 9.8 *4 / 32768;
            break;
        case 0x02:
            accel = ((float)raw_accel) * 9.8 *8 / 32768;
            break;
        case 0x03:
            accel = ((float)raw_accel) * 9.8 *16 / 32768;
            break;
        default:
            accel = 0;
    }
    return accel;
}

// IIC����д
// addr:������ַ
// reg:�Ĵ�����ַ
// len:д�볤��
// buf:������
//����ֵ:0,����
//     ����,�������
u8 IMU_WriteLen(u8 addr, u8 reg, u8 len, u8 *buf)
{
    u8 i = 0;

    I2C_AcknowledgeConfig(I2C2, ENABLE);
    I2C_GenerateSTART(I2C2, ENABLE);

    while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT))
        ;
    I2C_Send7bitAddress(I2C2, ((addr << 1) | 0), I2C_Direction_Transmitter); //����������ַ+д����

    while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
        ; //�ȴ�Ӧ��

    while (I2C_GetFlagStatus(I2C2, I2C_FLAG_TXE) == RESET)
        ;
    I2C_SendData(I2C2, reg); //д�Ĵ�����ַ

    while (i < len)
    {
        if (I2C_GetFlagStatus(I2C2, I2C_FLAG_TXE) != RESET)
        {
            I2C_SendData(I2C2, buf[i]); //��������
            i++;
        }
    }
    //    while( !I2C_CheckEvent( I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED ) );
    while (I2C_GetFlagStatus(I2C2, I2C_FLAG_TXE) == RESET)
        ;

    I2C_GenerateSTOP(I2C2, ENABLE);

    return 0;
}

// IIC������
// addr:������ַ
// reg:Ҫ��ȡ�ļĴ�����ַ
// len:Ҫ��ȡ�ĳ���
// buf:��ȡ�������ݴ洢��
//����ֵ:0,����
//     ����,�������
u8 IMU_ReadLen(u8 addr, u8 reg, u8 len, u8 *buf)
{
    u8 i = 0;

    I2C_AcknowledgeConfig(I2C2, ENABLE);
    I2C_GenerateSTART(I2C2, ENABLE);

    while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT))
        ;
    I2C_Send7bitAddress(I2C2, (addr << 1) | 0X00, I2C_Direction_Transmitter); //����������ַ+д����

    while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
        ; //�ȴ�Ӧ��

    I2C_SendData(I2C2, reg); //д�Ĵ�����ַ

    I2C_GenerateSTART(I2C2, ENABLE);
    while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT))
        ;

    I2C_Send7bitAddress(I2C2, ((addr << 1) | 0x01), I2C_Direction_Receiver); //����������ַ+������
    while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))
        ; //�ȴ�Ӧ��

    while (i < len)
    {
        if (I2C_GetFlagStatus(I2C2, I2C_FLAG_RXNE) != RESET)
        {
            if (i == (len - 2))
            {
                I2C_AcknowledgeConfig(I2C2, DISABLE);
                buf[i] = I2C_ReceiveData(I2C2); //������,����nACK
            }
            else
            {
                buf[i] = I2C_ReceiveData(I2C2); //������,����ACK
            }
            i++;
        }
    }

    I2C_GenerateSTOP(I2C2, ENABLE); //����һ��ֹͣ����

    return 0;
}

// IICдһ���ֽ�
// reg:�Ĵ�����ַ
// data:����
//����ֵ:0,����
//     ����,�������
u8 IMU_WriteByte(u8 reg, u8 data)
{

    I2C_AcknowledgeConfig(I2C2, ENABLE);
    I2C_GenerateSTART(I2C2, ENABLE);

    while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT))
        ;
    I2C_Send7bitAddress(I2C2, ((MPU_ADDR << 1) | 0), I2C_Direction_Transmitter); //����������ַ+д����

    while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
        ; //�ȴ�Ӧ��

    while (I2C_GetFlagStatus(I2C2, I2C_FLAG_TXE) == RESET)
        ;
    I2C_SendData(I2C2, reg); //д�Ĵ�����ַ

    while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
        ; //�ȴ�Ӧ��

    while (I2C_GetFlagStatus(I2C2, I2C_FLAG_TXE) == RESET)
        ;
    I2C_SendData(I2C2, data); //��������

    I2C_GenerateSTOP(I2C2, ENABLE);

    return 0;
}

// IIC��һ���ֽ�
// reg:�Ĵ�����ַ
//����ֵ:����������
u8 IMU_ReadByte(u8 reg)
{
    u8 res;

    I2C_AcknowledgeConfig(I2C2, ENABLE);
    I2C_GenerateSTART(I2C2, ENABLE);

    while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT))
        ;
    I2C_Send7bitAddress(I2C2, (MPU_ADDR << 1) | 0X00, I2C_Direction_Transmitter); //����������ַ+д����

    while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
        ;                    //�ȴ�Ӧ��
    I2C_SendData(I2C2, reg); //д�Ĵ�����ַ

    while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
        ; //�ȴ�Ӧ��

    I2C_GenerateSTART(I2C2, ENABLE);
    while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT))
        ;

    I2C_Send7bitAddress(I2C2, ((MPU_ADDR << 1) | 0x01), I2C_Direction_Receiver); //����������ַ+������
    while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))
        ; //�ȴ�Ӧ��

    while (1)
    {
        if (I2C_GetFlagStatus(I2C2, I2C_FLAG_RXNE) != RESET)
        {
            res = I2C_ReceiveData(I2C2); //������,����nACK
            break;
        }
    }

    I2C_GenerateSTOP(I2C2, ENABLE); //����һ��ֹͣ����

    return res;
}
