//
// Created by spaaaaace on 2026/4/22.
//

#ifndef INC_26BASE_FRAMEWORK_IR_WS2812_H
#define INC_26BASE_FRAMEWORK_IR_WS2812_H

#include "bsp_spi.h"

#define WS2812_MAX_LED  32

typedef struct
{
    SPI_Init_Config_s spi_config;
    SPIInstance *spi_instance;
    uint16_t led_num;

    uint8_t rgb_buf[WS2812_MAX_LED * 3];   // 原始RGB
    uint8_t spi_buf[WS2812_MAX_LED * 24];   // 编码后SPI数据

}WS2812Instance;

/**
 * @brief 初始化WS2812
 */
WS2812Instance *WS2812_Init(SPI_HandleTypeDef *hspi, uint16_t led_num);

/**
 * @brief 设置单个LED颜色
 */
void WS2812_SetPixel(WS2812Instance *ws, uint16_t index, uint8_t r, uint8_t g, uint8_t b);

/**
 * @brief 设置全部LED为同一颜色
 */
void WS2812_SetAll(WS2812Instance *ws, uint8_t r, uint8_t g, uint8_t b);

/**
 * @brief 刷新LED（发送数据）
 */
void WS2812_Refresh();

/**
 * @brief 清屏
 */
void WS2812_Clear(WS2812Instance *ws);

/**
 * @brief 工具函数，色相转RGB
 */
void WS2812_HueToRGB(uint8_t hue, uint8_t *r, uint8_t *g, uint8_t *b);

#endif //INC_26BASE_FRAMEWORK_IR_WS2812_H