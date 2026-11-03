/**
  ******************************************************************************
  * @file    pid.c
  * @brief   增量式 PID 实现
  *
  *   增量式公式：
  *     Δu = Kp·(e[k] − e[k−1]) + Ki·e[k]·dt + Kd·(e[k] − 2e[k−1] + e[k−2]) / dt
  *     u += Δu
  *
  *   三项的物理含义：
  *     Kp 项：误差"变化量" → 越大响应越快，但会振荡
  *     Ki 项：误差"累积"   → 消除稳态误差，但太大超调
  *     Kd 项：误差"加速度" → 抑制超调，但放大噪声（速度环一般不加）
  ******************************************************************************
  */
#include "pid.h"

void PID_Init(PID_t *p, float kp, float ki, float kd,
              float dt, float out_min, float out_max)
{
    p->kp = kp;
    p->ki = ki;
    p->kd = kd;
    p->dt = dt;
    p->e1 = 0.0f;
    p->e2 = 0.0f;
    p->out = 0.0f;
    p->out_min = out_min;
    p->out_max = out_max;
    p->first = 1;
}

void PID_Reset(PID_t *p, float out)
{
    p->e1 = 0.0f;
    p->e2 = 0.0f;
    p->out = out;
    p->first = 1;
}

/**
  * @brief  更新一次 PID
  * @param  target 目标速度（mm/s）
  * @param  actual 实际速度（mm/s）
  * @retval 输出（可直接当 CCR 用，已限幅）
  */
float PID_Update(PID_t *p, float target, float actual)
{
    float e = target - actual;
    float du;

    if (p->first) {
        /* 第一次进来，把历史误差清零，避免 Δu 里出现一个假的大冲击 */
        p->e1 = e;
        p->e2 = e;
        p->first = 0;
    }

    du = p->kp * (e - p->e1)
       + p->ki * e * p->dt
       + p->kd * (e - 2.0f * p->e1 + p->e2) / p->dt;

    p->e2 = p->e1;
    p->e1 = e;

    /* 抗积分饱和：宁可少加，不要加过头 */
    if ((p->out >= p->out_max && du > 0.0f) ||
        (p->out <= p->out_min && du < 0.0f)) {
        du = 0.0f;
    }

    p->out += du;

    if (p->out > p->out_max) p->out = p->out_max;
    if (p->out < p->out_min) p->out = p->out_min;

    return p->out;
}
