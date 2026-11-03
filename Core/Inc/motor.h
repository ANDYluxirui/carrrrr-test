/**
  ******************************************************************************
  * @file    motor.h
  * @brief   TB6612 电机驱动（带死区补偿）
  *
  *   引脚（按你现在的接法）：
  *     左轮：AIN1=PB0  AIN2=PB1  PWMA=PA6(TIM3_CH1)
  *     右轮：BIN1=PB10 BIN2=PB11 PWMB=PA7(TIM3_CH2)
  ******************************************************************************
  */
#ifndef __MOTOR_H
#define __MOTOR_H

#include <stdint.h>
#include "main.h"

void Motor_Init(void);
void Motor_SetLeft(int32_t duty);    /* -PWM_ARR ~ +PWM_ARR */
void Motor_SetRight(int32_t duty);
void Motor_Stop(void);               /* 高阻停止（能自由推行）*/
void Motor_Brake(void);              /* 刹车（锁死）*/

extern int32_t g_duty_l;             /* 实际下发的占空比，遥测用 */
extern int32_t g_duty_r;

#endif /* __MOTOR_H */
