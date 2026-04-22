//
// Created by LIMBO on 2026/4/21.
//

#ifndef INC_26IRCCSOFTWARE_GRAYSCALE_H
#define INC_26IRCCSOFTWARE_GRAYSCALE_H

#include "main.h"
#include "robot_def.h"

// 灰度传感器数据结构体
typedef struct {
    uint16_t sensor_values[8];  // 8个灰度传感器模拟值/数字值
    uint8_t data_mode;          // 数据模式: 0=A模式(模拟), 1=D模式(数字)
    uint8_t valid;             // 数据有效标志
    uint8_t new_package_flag;  // 新数据包标志
} GrayscaleData_t;

/**
 * @brief 灰度传感器初始化
 * @param usart_handle 串口硬件句柄
 * @return 灰度传感器数据指针
 */
GrayscaleData_t *GrayscaleInit(UART_HandleTypeDef *usart_handle);

/**
 * @brief 发送控制命令给灰度传感器
 * @param calib 校准模式 (0/1)
 * @param a_mode 模拟模式 (0/1)
 * @param d_mode 数字模式 (0/1)
 */
void GrayscaleSendControl(uint8_t calib, uint8_t a_mode, uint8_t d_mode);

/**
 * @brief 发送查询命令给灰度传感器 (默认模拟模式)
 */
void GrayscaleSendQuery();

/**
 * @brief 获取灰度传感器数据
 * @return 灰度传感器数据指针
 */
GrayscaleData_t *GrayscaleGetData();

/**
 * @brief 检查灰度传感器是否在线
 * @return 1-在线 0-离线
 */
uint8_t GrayscaleIsOnline();

/**
 * @brief 清除新数据包标志
 */
void GrayscaleClearNewPackageFlag();

#endif //INC_26IRCCSOFTWARE_GRAYSCALE_H
