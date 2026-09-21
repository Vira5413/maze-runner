/*
 * mpu6050.h
 *
 *  Created on: Aug 28, 2026
 *      Author: viraj
 */

#ifndef MPU6050_H
#define MPU6050_H

#include "main.h"

void MPU6050_Init(void);
int16_t MPU6050_ReadGyroZ_Raw(void);
float MPU6050_ReadGyroZ_DPS(void);
uint8_t MPU6050_ReadRegister(uint8_t reg);

#endif
