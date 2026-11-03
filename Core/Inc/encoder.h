/**
  ******************************************************************************
  * @file    encoder.h
  * @brief   编码器测速（TIM2 左轮 / TIM4 右轮，都是编码器模式 TI12）
  ******************************************************************************
  */
#ifndef __ENCODER_H
#define __ENCODER_H

#include <stdint.h>

extern float g_speed_l;   /* 左轮速度 mm/s */
extern float g_speed_r;   /* 右轮速度 mm/s */
extern int32_t g_cnt_l;   /* 左轮累计计数（可当里程用）*/
extern int32_t g_cnt_r;

void  Encoder_Init(void);
void  Encoder_Update(void);       /* 每 10ms 调一次 */
void  Encoder_Reset(void);

#endif /* __ENCODER_H */
