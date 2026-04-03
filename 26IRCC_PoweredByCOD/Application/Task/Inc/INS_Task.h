/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : INS_Task.h
  * @brief          : INS 任务头文件
  * @author         : Yan Yuanbin
  * @date           : 2023/04/27
  * @version        : v1.0
  ******************************************************************************
  * @attention      : None
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef INS_TASK_H
#define INS_TASK_H

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"

/* Exported types ------------------------------------------------------------*/
/**
 * @brief INS 状态信息结构体。
 */
typedef struct 
{
  /* 欧拉角(角度制) */
	float Pitch_Angle;
	float Yaw_Angle; // 目标偏航角，单位度
	float Yaw_TolAngle; // 累计航向角，单位度
	float Roll_Angle;

  /* 角速度(角度制) */
  float Pitch_Gyro;
  float Yaw_Gyro;
  float Roll_Gyro;

  /* 原始姿态与传感器数据(弧度制/原始单位) */
  float Angle[3];
	float Gyro[3];	
	float Accel[3];
	
  /* 航向角跨圈累计辅助变量 */
	float Last_Yaw_Angle;
	int16_t YawRoundCount;

}INS_Info_Typedef;

/* Externs---------------------------------------------------------*/
extern INS_Info_Typedef INS_Info; 

/**
 * @brief INS 任务入口函数。
 * @param argument RTOS 线程参数，未使用。
 */
void INS_Task(void const * argument);

#endif //INS_TASK_H
