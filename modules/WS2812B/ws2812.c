//
// Created by spaaaaace on 2026/4/22.
//

#include "ws2812.h"
#include <stdlib.h>
#include <string.h>

#define MAX_LED_LIST 8

#define WS2812_LOW   0x60   // 0 -> 0110 0000
#define WS2812_HIGH  0x78   // 1 -> 0111 1000

WS2812Instance *ws2812list[MAX_LED_LIST];
static int idx = 0;

void WS2812_HueToRGB(uint8_t hue, uint8_t *r, uint8_t *g, uint8_t *b)
{
    uint8_t region = hue / 43;       // 0~5
    uint8_t remainder = (hue - (region * 43)) * 6;

    uint8_t p = 0;
    uint8_t q = 255 - remainder;
    uint8_t t = remainder;

    switch (region)
    {
    case 0:
        *r = 255; *g = t;   *b = 0;
        break;
    case 1:
        *r = q;   *g = 255; *b = 0;
        break;
    case 2:
        *r = 0;   *g = 255; *b = t;
        break;
    case 3:
        *r = 0;   *g = q;   *b = 255;
        break;
    case 4:
        *r = t;   *g = 0;   *b = 255;
        break;
    default:
        *r = 255; *g = 0;   *b = q;
        break;
    }
}

/**
 * @brief 设置单个LED
 */
void WS2812_SetPixel(WS2812Instance *ws, uint16_t index, uint8_t r, uint8_t g, uint8_t b)
{
    if (index >= ws->led_num) return;

    ws->rgb_buf[index * 3 + 0] = r;
    ws->rgb_buf[index * 3 + 1] = g;
    ws->rgb_buf[index * 3 + 2] = b;
}

/**
 * @brief 设置全部LED
 */
void WS2812_SetAll(WS2812Instance *ws, uint8_t r, uint8_t g, uint8_t b)
{
    for (int i = 0; i < ws->led_num; i++)
    {
        WS2812_SetPixel(ws, i, r, g, b);
    }
}

/**
 * @brief 清屏
 */
void WS2812_Clear(WS2812Instance *ws)
{
    WS2812_SetAll(ws, 0, 0, 0);
}

/**
 * @brief 编码（8bit方案）
 */
static void WS2812_Encode(WS2812Instance *ws)
{
    for (int i = 0; i < ws->led_num; i++)
    {
        uint8_t r = ws->rgb_buf[i * 3 + 0];
        uint8_t g = ws->rgb_buf[i * 3 + 1];
        uint8_t b = ws->rgb_buf[i * 3 + 2];

        // 每个LED占 24 byte
        uint8_t *p = &ws->spi_buf[i * 24];

        // GRB顺序
        uint8_t color[3] = {g, r, b};

        int idx_buf = 0;

        for (int c = 0; c < 3; c++)
        {
            for (int bit = 7; bit >= 0; bit--)
            {
                if (color[c] & (1 << bit))
                    p[idx_buf++] = WS2812_HIGH;
                else
                    p[idx_buf++] = WS2812_LOW;
            }
        }
    }
}

/**
 * @brief 刷新LED
 */
void WS2812_Refresh()
{
    for (int i = 0; i < idx; i++)
    {
        WS2812Instance *ws = ws2812list[i];
        WS2812_Encode(ws);

        uint16_t len = ws->led_num * 24;

        SPITransmit(ws->spi_instance, ws->spi_buf, len);

        // reset > 50us
        uint8_t reset[80] = {0};
        SPITransmit(ws->spi_instance, reset, 80);
    }
}

/**
 * @brief 初始化
 */
WS2812Instance *WS2812_Init(SPI_HandleTypeDef *hspi, uint16_t led_num)
{
    WS2812Instance *instance = (WS2812Instance*)malloc(sizeof(WS2812Instance));
    memset(instance, 0, sizeof(WS2812Instance));

    if (led_num >= WS2812_MAX_LED)
        instance->led_num = WS2812_MAX_LED;
    else
        instance->led_num = led_num;

    // SPI配置
    instance->spi_config.spi_handle = hspi;

    //WS2812 不需要 CS，这里随便填但不会用
    instance->spi_config.GPIOx = GPIOA;
    instance->spi_config.cs_pin = GPIO_PIN_4;

    instance->spi_config.spi_work_mode = SPI_BLOCK_MODE;

    instance->spi_instance = SPIRegister(&instance->spi_config);

    ws2812list[idx++] = instance;

    return instance;
}