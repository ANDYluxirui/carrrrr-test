/**
  ******************************************************************************
  * @file    oled.c
  * @brief   SSD1306 OLED 驱动（显存式，8x8 字模）
  *
  *   显存排布与 SSD1306 的 page 格式一致：
  *     fb[page][x]，每个字节是一列 8 个像素（bit0 在上）
  *   字模是「按行」存的（bit0 在左），写显存时要转置：
  *     fb[p][x] 的第 r 位 = 字模第 r 行的第 x 位
  ******************************************************************************
  */
#include "oled.h"
#include "i2c_bus.h"
#include "oledfont.h"
#include <string.h>

#define OLED_ADDR   0x78        /* 0x3C << 1 */

static uint8_t fb[8][128];
static uint8_t s_ok = 0;

/* ------------------------------------------------------------------ */
static void wr_cmd(uint8_t c)
{
    uint8_t b[2];
    b[0] = 0x00;          /* 控制字节：后续是命令 */
    b[1] = c;
    I2C_Write(OLED_ADDR, b, 2);
}

static void wr_data(const uint8_t *d, uint16_t n)
{
    static uint8_t buf[129];
    if (n > 128) n = 128;
    buf[0] = 0x40;        /* 控制字节：后续是显存数据 */
    memcpy(&buf[1], d, n);
    I2C_Write(OLED_ADDR, buf, (uint16_t)(n + 1));
}

/* ------------------------------------------------------------------ */
void OLED_Init(void)
{
    static const uint8_t seq[] = {
        0xAE,        /* 关显示 */
        0x20, 0x00,  /* 水平寻址 */
        0xB0,        /* page 0 */
        0xC8,        /* COM 扫描方向：上下翻转（常见模块用 C8） */
        0x00,        /* 低列地址 */
        0x10,        /* 高列地址 */
        0x40,        /* 起始行 0 */
        0x81, 0xCF,  /* 对比度 */
        0xA1,        /* 段重映射：左右翻转 */
        0xA6,        /* 正常显示 */
        0xA8, 0x3F,  /* 复用率 1/64 */
        0xA4,        /* 显示跟随显存 */
        0xD3, 0x00,  /* 显示偏移 0 */
        0xD5, 0x80,  /* 时钟分频 */
        0xD9, 0xF1,  /* 预充电周期 */
        0xDA, 0x12,  /* COM 引脚配置 */
        0xDB, 0x40,  /* VCOMH */
        0x8D, 0x14,  /* 电荷泵使能 */
        0xAF         /* 开显示 */
    };
    uint16_t i;

    I2C_Bus_Init();
    HAL_Delay(50);

    if (!I2C_Probe(0x3C)) { s_ok = 0; return; }

    for (i = 0; i < sizeof(seq); i++) wr_cmd(seq[i]);

    OLED_Clear();
    OLED_Flush();
    s_ok = 1;
}

uint8_t OLED_IsOk(void) { return s_ok; }

/* ------------------------------------------------------------------ */
void OLED_Clear(void)
{
    memset(fb, 0, sizeof(fb));
}

void OLED_Flush(void)
{
    uint8_t p;

    if (!s_ok) return;

    for (p = 0; p < 8; p++) OLED_FlushPage(p);
}

/**
  * @brief 只刷新一页（128 字节 ≈ 2.6ms @100kHz）
  *        主循环里轮流刷各页，避免一次性 1024 字节阻塞控制环 90ms
  */
void OLED_FlushPage(uint8_t page)
{
    if (!s_ok || page > 7) return;

    wr_cmd((uint8_t)(0xB0 | page));
    wr_cmd(0x00);
    wr_cmd(0x10);
    wr_data(fb[page], 128);
}

/* ------------------------------------------------------------------ */
static void put_char(uint8_t page, uint8_t col, char ch)
{
    const uint8_t *g;
    uint8_t x, r, byte;

    if (page > 7 || col > 15) return;
    if ((uint8_t)ch > 127) ch = '?';
    if ((uint8_t)ch < 32)  ch = ' ';

    g = oled_font8x8[(uint8_t)ch];

    for (x = 0; x < 8; x++) {
        byte = 0;
        for (r = 0; r < 8; r++) {
            if ((g[r] >> x) & 0x01) byte |= (uint8_t)(1u << r);
        }
        fb[page][col * 8 + x] = byte;
    }
}

void OLED_Str(uint8_t page, uint8_t col, const char *s)
{
    while (*s && col < 16) {
        put_char(page, col++, *s++);
    }
}

void OLED_Int(uint8_t page, uint8_t col, int32_t v, uint8_t w)
{
    char buf[12];
    int  i = 0, k;
    uint8_t neg = 0;

    if (v < 0) { neg = 1; v = -v; }

    do { buf[i++] = (char)('0' + (v % 10)); v /= 10; } while (v > 0);
    if (neg) buf[i++] = '-';

    /* 右对齐：先在左边补空格 */
    for (k = i; k < w && col < 16; k++) put_char(page, col++, ' ');
    while (i > 0 && col < 16) put_char(page, col++, buf[--i]);
}
