/**
  ******************************************************************************
  * @file    oled.h
  * @brief   0.96" OLED（SSD1306，128x64，I2C 地址 0x3C）
  *          与 MPU6050 共用 PB8/PB9 这条 I2C 总线
  ******************************************************************************
  */
#ifndef __OLED_H
#define __OLED_H

#include <stdint.h>

void    OLED_Init(void);                       /* 返回前会清屏 */
uint8_t OLED_IsOk(void);

void    OLED_Clear(void);
void    OLED_Flush(void);                      /* 全屏刷新（开机用） */
void    OLED_FlushPage(uint8_t page);          /* 只刷一页（运行时用） */

/* 在第 page 行（0~7）、第 col 个字符位（0~15）打印字符串 */
void    OLED_Str(uint8_t page, uint8_t col, const char *s);

/* 在第 page 行、第 col 个字符位打印一个右对齐整数（占 w 个字符） */
void    OLED_Int(uint8_t page, uint8_t col, int32_t v, uint8_t w);

#endif /* __OLED_H */
