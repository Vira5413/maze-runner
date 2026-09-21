/*
 * mpu6050.c
 *
 *  Created on: Aug 28, 2026
 *      Author: viraj
 */


#include "mpu6050.h"

#define SDA_PIN      GPIO_PIN_14
#define SCL_PIN      GPIO_PIN_15
#define I2C_PORT     GPIOC
#define MPU6050_ADDR 0x68
#define GyroZ_calib_samples 200

static float GyroZ_calib_value = 0;

static void MPU_DelayUs(uint32_t us)
{
  uint32_t start = DWT->CYCCNT;
  uint32_t cycles = us * (SystemCoreClock / 1000000);
  while ((DWT->CYCCNT - start) < cycles);
}

static void SDA_HIGH(void) { HAL_GPIO_WritePin(I2C_PORT, SDA_PIN, GPIO_PIN_SET); }
static void SDA_LOW(void)  { HAL_GPIO_WritePin(I2C_PORT, SDA_PIN, GPIO_PIN_RESET); }
static void SCL_HIGH(void) { HAL_GPIO_WritePin(I2C_PORT, SCL_PIN, GPIO_PIN_SET); }
static void SCL_LOW(void)  { HAL_GPIO_WritePin(I2C_PORT, SCL_PIN, GPIO_PIN_RESET); }
static uint8_t SDA_READ(void) { return HAL_GPIO_ReadPin(I2C_PORT, SDA_PIN); }

static void I2C_Start(void)
{
  SDA_HIGH(); SCL_HIGH(); MPU_DelayUs(5);
  SDA_LOW();  MPU_DelayUs(5);
  SCL_LOW();  MPU_DelayUs(5);
}

static void I2C_Stop(void)
{
  SDA_LOW();  MPU_DelayUs(5);
  SCL_HIGH(); MPU_DelayUs(5);
  SDA_HIGH(); MPU_DelayUs(5);
}

static uint8_t I2C_WriteByte(uint8_t data)
{
  for (int i = 0; i < 8; i++)
  {
    if (data & 0x80) SDA_HIGH(); else SDA_LOW();
    MPU_DelayUs(3);
    SCL_HIGH(); MPU_DelayUs(5);
    SCL_LOW();  MPU_DelayUs(3);
    data <<= 1;
  }
  SDA_HIGH(); // did high to read back if line is pulled low by slave
  MPU_DelayUs(3);
  SCL_HIGH(); MPU_DelayUs(3);
  uint8_t ack = !SDA_READ();
  SCL_LOW();
  return ack;
}

static uint8_t I2C_ReadByte(uint8_t sendAck)
{
  uint8_t data = 0;
  SDA_HIGH();
  for (int i = 0; i < 8; i++)
  {
    SCL_HIGH(); MPU_DelayUs(5);
    data = (data << 1) | SDA_READ();
    SCL_LOW();  MPU_DelayUs(3);
  }
  if (sendAck) SDA_LOW(); else SDA_HIGH();
  MPU_DelayUs(3);
  SCL_HIGH(); MPU_DelayUs(5);
  SCL_LOW();
  SDA_HIGH();
  return data;
}

void MPU6050_Init(void)
{
  HAL_Delay(200);
  I2C_Start();
  I2C_WriteByte(MPU6050_ADDR << 1);
  I2C_WriteByte(0x6B);
  I2C_WriteByte(0x00);
  I2C_Stop();

  //configure gyro full-scale range: FS_SEL=1 -> ±500 dps
  I2C_Start();
  I2C_WriteByte(MPU6050_ADDR << 1);
  I2C_WriteByte(0x1B);   // GYRO_CONFIG register
  I2C_WriteByte(0x08);   // FS_SEL bits [4:3] = 01 -> ±500 dps
  I2C_Stop();

  //uint8_t gyroConfig = MPU6050_ReadRegister(0x1B);
  // caliberation average
  float sum = 0;
  for (int i = 0; i < GyroZ_calib_samples; i++)
  {
	  sum += MPU6050_ReadGyroZ_Raw() / 65.5f ;
	  HAL_Delay(20);
  }
  GyroZ_calib_value = sum / GyroZ_calib_samples;
}

int16_t MPU6050_ReadGyroZ_Raw(void)
{
  uint8_t hi, lo;

  I2C_Start();
  I2C_WriteByte(MPU6050_ADDR << 1);
  I2C_WriteByte(0x47);
  I2C_Start();
  I2C_WriteByte((MPU6050_ADDR << 1) | 1);
  hi = I2C_ReadByte(1);
  lo = I2C_ReadByte(0);
  I2C_Stop();

  return (int16_t)((hi << 8) | lo);
}

float MPU6050_ReadGyroZ_DPS(void)
{
  //return ((MPU6050_ReadGyroZ_Raw() / 16.4f) - GyroZ_calib_value);
  return ((float)(MPU6050_ReadGyroZ_Raw() / 65.5f) - GyroZ_calib_value);
}

uint8_t MPU6050_ReadRegister(uint8_t reg)
{
  uint8_t data;
  I2C_Start();
  I2C_WriteByte(MPU6050_ADDR << 1);
  I2C_WriteByte(reg);
  I2C_Start();
  I2C_WriteByte((MPU6050_ADDR << 1) | 1);
  data = I2C_ReadByte(0);   // NACK after the single byte, signals "done reading"
  I2C_Stop();
  return data;
}
