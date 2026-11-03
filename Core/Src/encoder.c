/**
  ******************************************************************************
  * @file    encoder.c
  * @brief   编码器测速
  *
  *   原理：
  *     TIM 编码器模式（TI12）= 硬件四倍频，A/B 两相的上升沿下降沿全计数
  *     每 10ms 读一次计数值，两次相减 = 这 10ms 走了多少个脉冲
  *
  *   换算：
  *     输出轴一圈 = ENCODER_CPR 个计数 = 轮子走 π·D 毫米
  *     v(mm/s) = Δcount / CPR × π·D / dt
  *
  *   注意 TIM2 是 32 位、TIM4 是 16 位：
  *     TIM4 溢出时会从 65535 跳回 0，所以差值要按 int16_t 解释
  ******************************************************************************
  */
#include "encoder.h"
#include "app_config.h"
#include "main.h"

extern TIM_HandleTypeDef htim2;   /* 左轮：PA0/PA1 */
extern TIM_HandleTypeDef htim4;   /* 右轮：PB6/PB7 */

float   g_speed_l = 0.0f;
float   g_speed_r = 0.0f;
int32_t g_cnt_l   = 0;
int32_t g_cnt_r   = 0;

static int32_t s_last_l = 0;
static int32_t s_last_r = 0;

/* 每个计数对应多少 mm/s（在 10ms 周期下）*/
#define MM_PER_COUNT   (3.14159265f * WHEEL_DIAMETER_MM / ENCODER_CPR)
#define SPEED_SCALE    (MM_PER_COUNT / CTRL_DT_S)

void Encoder_Init(void)
{
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

    __HAL_TIM_SET_COUNTER(&htim2, 0);
    __HAL_TIM_SET_COUNTER(&htim4, 0);

    s_last_l = 0;
    s_last_r = 0;
    g_cnt_l  = 0;
    g_cnt_r  = 0;
    g_speed_l = 0.0f;
    g_speed_r = 0.0f;
}

/**
  * @brief 每 10ms 调一次，更新 g_speed_l / g_speed_r
  */
void Encoder_Update(void)
{
    int32_t now_l = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
    int32_t now_r = (int32_t)__HAL_TIM_GET_COUNTER(&htim4);

    /* TIM4 只有 16 位，用 int16_t 做差可以自动处理溢出回绕 */
    int16_t d_l = (int16_t)(now_l - s_last_l);
    int16_t d_r = (int16_t)(now_r - s_last_r);

    s_last_l = now_l;
    s_last_r = now_r;

    g_cnt_l += (int32_t)d_l * ENC_L_SIGN;
    g_cnt_r += (int32_t)d_r * ENC_R_SIGN;

    float v_l = (float)d_l * ENC_L_SIGN * SPEED_SCALE;
    float v_r = (float)d_r * ENC_R_SIGN * SPEED_SCALE;

    /* 一阶低通，压掉量化噪声（1 个计数 = 15mm/s 的台阶）*/
    g_speed_l = g_speed_l * (1.0f - SPD_LPF_ALPHA) + v_l * SPD_LPF_ALPHA;
    g_speed_r = g_speed_r * (1.0f - SPD_LPF_ALPHA) + v_r * SPD_LPF_ALPHA;
}

void Encoder_Reset(void)
{
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    __HAL_TIM_SET_COUNTER(&htim4, 0);
    s_last_l = 0;
    s_last_r = 0;
    g_cnt_l  = 0;
    g_cnt_r  = 0;
    g_speed_l = 0.0f;
    g_speed_r = 0.0f;
}
