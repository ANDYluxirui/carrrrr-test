/**
  ******************************************************************************
  * @file    mpu6050.h
  * @brief   MPU6050 陀螺仪驱动（只用 Z 轴角速度做航向）
  *
  *   接线（I2C1 重映射，避开已被编码器占用的 PB6/PB7）：
  *     PB8 → SCL
  *     PB9 → SDA
  *     3.3V → VCC
  *     GND  → GND
  ******************************************************************************
  */
#ifndef __MPU6050_H
#define __MPU6050_H

#include <stdint.h>

#define MPU6050_ADDR        0xD0    /* 7 位地址 0x68，左移一位 */

extern uint8_t g_mpu_ok;            /* 1 = 检测到模块 */
extern float   g_gyro_bias;         /* 零偏（LSB），上电标定得到 */

uint8_t MPU6050_Init(void);         /* 返回 1 表示成功 */
void    MPU6050_Calibrate(void);    /* 静止标定零偏，约 1 秒 */
float   MPU6050_ReadGyroZ(void);    /* 返回 °/s（已扣零偏） */
void    MPU6050_ReadAll(int16_t *ax, int16_t *ay, int16_t *az,
                        int16_t *gx, int16_t *gy, int16_t *gz);

#endif /* __MPU6050_H */
