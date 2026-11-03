/**
  ******************************************************************************
  * @file    i2c_bus.c
  * @brief   软件 I2C 实现（开漏输出 + 位操作）
  *
  *   时序：约 5µs 一个半周期 → 100kHz 左右，杜邦线上很稳。
  *   开漏模式下写 1 就是释放总线，可以直接读回 SDA 做应答检测。
  ******************************************************************************
  */
#include "i2c_bus.h"

static uint8_t s_inited = 0;

/* 半周期延时：72MHz 下约 2.5µs */
static void dly(void)
{
    volatile uint32_t i = 22;
    while (i--) { __NOP(); }
}

#define SCL_H()   HAL_GPIO_WritePin(I2C_SCL_PORT, I2C_SCL_PIN, GPIO_PIN_SET)
#define SCL_L()   HAL_GPIO_WritePin(I2C_SCL_PORT, I2C_SCL_PIN, GPIO_PIN_RESET)
#define SDA_H()   HAL_GPIO_WritePin(I2C_SDA_PORT, I2C_SDA_PIN, GPIO_PIN_SET)
#define SDA_L()   HAL_GPIO_WritePin(I2C_SDA_PORT, I2C_SDA_PIN, GPIO_PIN_RESET)
#define SDA_READ() HAL_GPIO_ReadPin(I2C_SDA_PORT, I2C_SDA_PIN)

/* ------------------------------------------------------------------ */
void I2C_Bus_Init(void)
{
    GPIO_InitTypeDef g = {0};

    if (s_inited) return;

    I2C_SCL_CLK_ENABLE();
    I2C_SDA_CLK_ENABLE();

    g.Pin   = I2C_SCL_PIN;
    g.Mode  = GPIO_MODE_OUTPUT_OD;      /* 开漏：写 1 = 释放，可被从机拉低 */
    g.Pull  = GPIO_PULLUP;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(I2C_SCL_PORT, &g);

    g.Pin = I2C_SDA_PIN;
    HAL_GPIO_Init(I2C_SDA_PORT, &g);

    SCL_H();
    SDA_H();
    HAL_Delay(2);
    s_inited = 1;
}

/* ------------------------------------------------------------------ */
static void i2c_start(void)
{
    SDA_H(); dly();
    SCL_H(); dly();
    SDA_L(); dly();
    SCL_L(); dly();
}

static void i2c_stop(void)
{
    SDA_L(); dly();
    SCL_H(); dly();
    SDA_H(); dly();
}

/* 写一个字节，返回 0 = 收到应答 */
static uint8_t i2c_write_byte(uint8_t b)
{
    uint8_t i, ack;

    for (i = 0; i < 8; i++) {
        if (b & 0x80) SDA_H(); else SDA_L();
        b <<= 1;
        dly();
        SCL_H(); dly();
        SCL_L(); dly();
    }

    SDA_H(); dly();          /* 释放 SDA，读应答 */
    SCL_H(); dly();
    ack = SDA_READ();        /* 0 = ACK */
    SCL_L(); dly();

    return ack;
}

/* 读一个字节，ack = 1 表示回 ACK（还要继续读） */
static uint8_t i2c_read_byte(uint8_t ack)
{
    uint8_t i, b = 0;

    SDA_H();
    for (i = 0; i < 8; i++) {
        dly();
        SCL_H(); dly();
        b = (uint8_t)((b << 1) | (SDA_READ() ? 1 : 0));
        SCL_L(); dly();
    }

    if (ack) SDA_L(); else SDA_H();   /* 给从机应答 */
    dly();
    SCL_H(); dly();
    SCL_L(); dly();
    SDA_H();

    return b;
}

/* ------------------------------------------------------------------ */
uint8_t I2C_Write(uint8_t addr8, const uint8_t *buf, uint16_t len)
{
    uint16_t i;

    I2C_Bus_Init();
    i2c_start();
    if (i2c_write_byte(addr8) != 0) { i2c_stop(); return 1; }

    for (i = 0; i < len; i++) {
        if (i2c_write_byte(buf[i]) != 0) { i2c_stop(); return 2; }
    }
    i2c_stop();
    return 0;
}

uint8_t I2C_WriteReg(uint8_t addr8, uint8_t reg, const uint8_t *buf, uint16_t len)
{
    uint16_t i;

    I2C_Bus_Init();
    i2c_start();
    if (i2c_write_byte(addr8) != 0) { i2c_stop(); return 1; }
    if (i2c_write_byte(reg)    != 0) { i2c_stop(); return 2; }

    for (i = 0; i < len; i++) {
        if (i2c_write_byte(buf[i]) != 0) { i2c_stop(); return 3; }
    }
    i2c_stop();
    return 0;
}

uint8_t I2C_ReadReg(uint8_t addr8, uint8_t reg, uint8_t *buf, uint16_t len)
{
    uint16_t i;

    I2C_Bus_Init();
    i2c_start();
    if (i2c_write_byte(addr8) != 0) { i2c_stop(); return 1; }
    if (i2c_write_byte(reg)    != 0) { i2c_stop(); return 2; }

    i2c_start();                                  /* 重复起始 */
    if (i2c_write_byte((uint8_t)(addr8 | 0x01)) != 0) { i2c_stop(); return 3; }

    for (i = 0; i < len; i++) {
        buf[i] = i2c_read_byte((i + 1 < len) ? 1 : 0);
    }
    i2c_stop();
    return 0;
}

uint8_t I2C_Probe(uint8_t addr7)
{
    uint8_t ok;

    I2C_Bus_Init();
    i2c_start();
    ok = (i2c_write_byte((uint8_t)(addr7 << 1)) == 0);
    i2c_stop();
    return ok;
}
