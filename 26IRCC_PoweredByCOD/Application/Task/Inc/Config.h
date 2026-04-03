/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : Config.h
  * @brief          : 全局配置宏与常量
  * @author         : Yan Yuanbin
  * @date           : 2023/05/21
  * @version        : v1.0
  ******************************************************************************
  * @attention      : To be perfected
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "stdbool.h"
#include "stdlib.h"
#include "string.h"
#include "math.h"

/* General physics and mathematics constants ---------------------------------*/

/**
 * @brief 将变量限制在 [min, max] 范围内。
 * @note  第一个参数必须是可赋值左值。
 */
 
 #define VAL_LIMIT(x,min,max)  do{ \
                                    if ((x) > (max)) {(x) = (max);} \
                                    else if ((x) < (min)) {(x) = (min);} \
                                }while(0U)

/**
 * @brief 本地重力加速度常量(用于运动估计)。
 */
#define GravityAccel  9.718f

/**
 * @brief 角度转弧度比例系数。
 */
#define Angle_to_rad  0.01745329f

/**
 * @brief 弧度转角度比例系数。
 */
#define Rad_to_angle  57.2957732f

/**
  * @brief 自然常数 e
  */
#define Euler_Number 2.718281828459045f

/**
 * @brief 弧度转角度系数(180.f/PI)
 */
#define RadiansToDegrees 57.295779513f

/**
 * @brief 角度转弧度系数(PI/180.f)
 */
#define DegreesToRadians 0.01745329251f

/* Vision reslove constants -------------------------------------------------*/

/**
 * @brief  目标选择模式
 *         0: 选择偏航角最小装甲板
 *         1: 选择距离最近装甲板
 */
#define Yaw_Distance_Decision  0

/**
 * @brief 弹道系数
 * @note  17mm: 0.038
 *        42mm: 0.019
 */
#define Bullet_Coefficient  0.038f

/**
 * @brief 小装甲半宽
 */
#define LittleArmor_HalfWidth   0.07f

/**
 * @brief 大装甲半宽
 */
#define LargeArmor_HalfWidth   0.1175f

/* IMU reslove constants ---------------------------------------------------*/
/**
 * @brief BMI088 标定开关
 *        0: 禁用
 *        1: 启用
 */
#define IMU_Calibration_ENABLE  0U

/**
 * @brief 俯仰角索引
 */
#define IMU_ANGLE_INDEX_PITCH  2U
/**
 * @brief 偏航角索引
 */
#define IMU_ANGLE_INDEX_YAW   0U
/**
 * @brief 横滚角索引
 */
#define IMU_ANGLE_INDEX_ROLL   1U

/**
 * @brief 俯仰角速度索引
 */
#define IMU_GYRO_INDEX_PITCH  0U
/**
 * @brief 偏航角速度索引
 */
#define IMU_GYRO_INDEX_YAW   2U
/**
 * @brief 横滚角速度索引
 */
#define IMU_GYRO_INDEX_ROLL   1U

/**
 * @brief 俯仰方向加速度索引
 */
#define IMU_ACCEL_INDEX_PITCH  0U
/**
 * @brief 偏航方向加速度索引
 */
#define IMU_ACCEL_INDEX_YAW   2U
/**
 * @brief 横滚方向加速度索引
 */
#define IMU_ACCEL_INDEX_ROLL   1U

/* Remote reslove constants -----------------------------------------------*/
/**
 * @brief 遥控数据接收通道选择
 * @note  0: CAN
 *        1: USART
 */
#define REMOTE_FRAME_USART_CAN   0U

#endif //ROBOT_CONFIG_H


