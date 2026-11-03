/**
  ******************************************************************************
  * @file    mpu6050.c
  * @brief   MPU6050 驱动 —— I2C1 重映射到 PB8/PB9
  *
  *   为什么重映射：
  *     I2C1 默认引脚是 PB6/PB7，但这两个脚已经被 TIM4 编码器占用。
  *     STM32F103 支持把 I2C1 重映射到 PB8(SCL)/PB9(SDA)，
  *     这样不用改动编码器接线就能用硬件 I2C。
  ******************************************************************************
  */
#include "mpu6050.h"
#include "app_config.h"
#include "main.h"
#include "i2c_bus.h"

/* ---- MPU6050 寄存器 ---- */
#define REG_SMPLRT_DIV   0x19
#define REG_CONFIG       0x1A
#define REG_GYRO_CONFIG  0x1B
#define REG_ACCEL_CONFIG 0x1C
#define REG_ACCEL_XOUT_H 0x3B
#define REG_PWR_MGMT_1   0x6B
#define REG_WHO_AM_I     0x75

uint8_t g_mpu_ok   = 0;
float   g_gyro_bias = 0.0f;

/* ------------------------------------------------------------------ */
static uint8_t wr(uint8_t reg, uint8_t val)
{
    return (I2C_WriteReg(MPU6050_ADDR, reg, &val, 1) == 0);
}

static uint8_t rd(uint8_t reg, uint8_t *buf, uint16_t len)
{
    return (I2C_ReadReg(MPU6050_ADDR, reg, buf, len) == 0);
}

/* ------------------------------------------------------------------ */
uint8_t MPU6050_Init(void)
{
    uint8_t id = 0;

    I2C_Bus_Init();
    HAL_Delay(50);                        /* 等模块上电稳定 */

    if (!rd(REG_WHO_AM_I, &id, 1)) {
        g_mpu_ok = 0;
        return 0;
    }
    /* WHO_AM_I 一般读回 0x68；有些贴片返回 0x70/0x98，不做严格比对 */
    if (id == 0x00 || id == 0xFF) {
        g_mpu_ok = 0;
        return 0;
    }

    wr(REG_PWR_MGMT_1, 0x80);             /* 软复位 */
    HAL_Delay(50);
    wr(REG_PWR_MGMT_1, 0x01);             /* 唤醒，时钟源 PLL_X 轴陀螺 */
    wr(REG_SMPLRT_DIV, 9);                /* 采样率 1kHz/(1+9) = 100Hz */
    wr(REG_CONFIG, 0x03);                 /* DLPF 44Hz，滤掉电机振动 */
    wr(REG_GYRO_CONFIG, 0x08);            /* ±500 °/s */
    wr(REG_ACCEL_CONFIG, 0x00);           /* ±2g */

    g_mpu_ok = 1;
    return 1;
}

/* ------------------------------------------------------------------ */
void MPU6050_Calibrate(void)
{
    int32_t sum = 0;
    uint16_t i, n = GYRO_CALIB_N;
    uint8_t buf[2];

    if (!g_mpu_ok) return;

    /* 必须在车静止时调用 */
    for (i = 0; i < n; i++) {
        if (rd(0x47, buf, 2)) {                     /* GYRO_ZOUT_H */
            sum += (int16_t)((buf[0] << 8) | buf[1]);
        }
        HAL_Delay(1);
    }
    g_gyro_bias = (float)sum / (float)n;
}

/* ------------------------------------------------------------------ */
float MPU6050_ReadGyroZ(void)
{
    uint8_t buf[2];

    if (!g_mpu_ok) return 0.0f;
    if (!rd(0x47, buf, 2)) return 0.0f;            /* GYRO_ZOUT_H */

    int16_t raw = (int16_t)((buf[0] << 8) | buf[1]);
    /* GYRO_Z_SIGN：模块翻面安装时取 -1，把 Z 轴方向掰回来 */
    return ((float)raw - g_gyro_bias) / GYRO_LSB_PER_DPS * (float)GYRO_Z_SIGN;
}

/* ------------------------------------------------------------------ */
void MPU6050_ReadAll(int16_t *ax, int16_t *ay, int16_t *az,
                     int16_t *gx, int16_t *gy, int16_t *gz)
{
    uint8_t b[14];

    if (!g_mpu_ok || !rd(REG_ACCEL_XOUT_H, b, 14)) return;

    *ax = (int16_t)((b[0]  << 8) | b[1]);
    *ay = (int16_t)((b[2]  << 8) | b[3]);
    *az = (int16_t)((b[4]  << 8) | b[5]);
    *gz = (int16_t)((b[12] << 8) | b[13]);
    *gx = (int16_t)((b[8]  << 8) | b[9]);
    *gy = (int16_t)((b[10] << 8) | b[11]);
}
