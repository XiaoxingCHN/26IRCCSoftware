/* USER CODE BEGIN Header */
/**
	******************************************************************************
	* @file           : CAN_Task.h
	* @brief          : CAN 通信任务头文件
	* @author         : GrassFan Wang
	* @date           : 2025/01/22
	* @version        : v1.0
	******************************************************************************
	* @attention      : None
	******************************************************************************
	*/
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef CAN_TASK_H
#define CAN_TASK_H

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"

/**
 * @brief CAN 通信任务入口函数。
 * @param argument RTOS 线程参数，未使用。
 */
void CAN_Task(void const * argument);





#endif