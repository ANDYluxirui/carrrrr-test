/**
  ******************************************************************************
  * @file    motor.c
  * @brief   TB6612 驱动 + 死区补偿
  *
  *   真值表（每路）：
  *     IN1=1 IN2=0  → 正转
  *     IN1=0 IN2=1  → 反转
  *     IN1=1 IN2=1  → 刹车（绕组短路，轮子锁死）
  *     IN1=0 IN2=0  → 停止（输出高阻，轮子能自由转）
  *
  *   ★ 死区补偿为什么必要：
  *     电机有静摩擦，占空比小于 ~8% 时电流不足以启动。
  *     PID 算出 5% 的指令，轮子纹丝不动，积分项就会一直累积 → 要么不动，要么突然冲出去。
  *     所以实际下发时：|duty| 加上一个 PWM_DEADZONE 的偏置。
  ******************************************************************************
  */
#include "motor.h"
#include "app_config.h"

extern TIM_HandleTypeDef htim3;

int32_t g_duty_l = 0;
int32_t g_duty_r = 0;

void Motor_Init(void)
{
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);

    Motor_Stop();
}

/* 内部：把逻辑占空比转成实际 CCR（含死区补偿）*/
static int32_t apply_deadzone(int32_t duty, int sign)
{
    int32_t mag = duty * sign;      /* 换算到"正 = 前进" */

    if (mag > PWM_ARR)  mag = PWM_ARR;
    if (mag < -PWM_ARR) mag = -PWM_ARR;

    if (mag > -PWM_STOP_BAND && mag < PWM_STOP_BAND) {
        return 0;                    /* 落在停止带里：直接关断，防止蠕动 */
    }

    if (mag > 0) {
        mag += PWM_DEADZONE;
        if (mag > PWM_ARR) mag = PWM_ARR;
    } else {
        mag -= PWM_DEADZONE;
        if (mag < -PWM_ARR) mag = -PWM_ARR;
    }
    return mag;
}

void Motor_SetLeft(int32_t duty)
{
    int32_t d = apply_deadzone(duty, MOTOR_L_SIGN);

    if (d > 0) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
    } else if (d < 0) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
    }
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (d >= 0) ? d : -d);
    g_duty_l = d;
}

void Motor_SetRight(int32_t duty)
{
    int32_t d = apply_deadzone(duty, MOTOR_R_SIGN);

    if (d > 0) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);
    } else if (d < 0) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);
    }
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, (d >= 0) ? d : -d);
    g_duty_r = d;
}

void Motor_Stop(void)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0  | GPIO_PIN_1,  GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10 | GPIO_PIN_11, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
    g_duty_l = 0;
    g_duty_r = 0;
}

void Motor_Brake(void)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0  | GPIO_PIN_1,  GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10 | GPIO_PIN_11, GPIO_PIN_SET);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
    g_duty_l = 0;
    g_duty_r = 0;
}
