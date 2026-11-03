/**
  ******************************************************************************
  * @file    i2c_bus.h
  * @brief   软件 I2C（位操作），引脚可自由指定
  *
  *   为什么用软件 I2C：
  *     硬件 I2C1 只能用 PB6/PB7（被编码器占用）或 PB8/PB9（被别的外设占用），
  *     I2C2 又占 PB10/PB11（电机方向脚）。软件 I2C 可以放在任意两个空闲 GPIO 上。
  *
  *   引脚在 app_config.h 里改：
  *     #define I2C_SCL_PORT   GPIOA
  *     #define I2C_SCL_PIN    GPIO_PIN_11
  *     #define I2C_SDA_PORT   GPIOA
  *     #define I2C_SDA_PIN    GPIO_PIN_12
  ******************************************************************************
  */
#ifndef __I2C_BUS_H
#define __I2C_BUS_H

#include "main.h"

/* ==========================================================================
 * ★ 引脚配置：换 I2C 引脚只改这 4 行
 *    当前用 PA11(SCL) / PA12(SDA)，这两个脚空闲且在同一侧，好接线。
 *    如果要用别的脚，记得同时改下面的时钟使能宏。
 * ========================================================================== */
#define I2C_SCL_PORT          GPIOA
#define I2C_SCL_PIN           GPIO_PIN_11
#define I2C_SDA_PORT          GPIOA
#define I2C_SDA_PIN           GPIO_PIN_12
#define I2C_SCL_CLK_ENABLE()  __HAL_RCC_GPIOA_CLK_ENABLE()
#define I2C_SDA_CLK_ENABLE()  __HAL_RCC_GPIOA_CLK_ENABLE()

void    I2C_Bus_Init(void);

uint8_t I2C_Write(uint8_t addr8, const uint8_t *buf, uint16_t len);
uint8_t I2C_WriteReg(uint8_t addr8, uint8_t reg, const uint8_t *buf, uint16_t len);
uint8_t I2C_ReadReg(uint8_t addr8, uint8_t reg, uint8_t *buf, uint16_t len);
uint8_t I2C_Probe(uint8_t addr7);        /* 返回 1 = 有设备应答 */

#endif /* __I2C_BUS_H */
