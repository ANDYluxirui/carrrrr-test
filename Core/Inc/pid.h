/**
  ******************************************************************************
  * @file    pid.h
  * @brief   增量式 PID（速度环专用）
  *
  *   为什么用增量式而不是位置式？
  *     1) 不需要积分限幅：增量天然有界（Δu 小）
  *     2) 切换模式/换目标时不产生积分冲击
  *     3) 抗积分饱和简单：输出超限就丢弃这次增量
  ******************************************************************************
  */
#ifndef __PID_H
#define __PID_H

#include <stdint.h>

typedef struct {
    float kp, ki, kd;
    float dt;            /* 采样周期，秒 */
    float e1, e2;        /* e[k-1], e[k-2] */
    float out;           /* 当前输出（CCR 值） */
    float out_min, out_max;
    uint8_t first;       /* 首次调用标志（避免历史误差为 0 造成冲击）*/
} PID_t;

void  PID_Init(PID_t *p, float kp, float ki, float kd,
               float dt, float out_min, float out_max);
void  PID_Reset(PID_t *p, float out);
float PID_Update(PID_t *p, float target, float actual);

#endif /* __PID_H */
