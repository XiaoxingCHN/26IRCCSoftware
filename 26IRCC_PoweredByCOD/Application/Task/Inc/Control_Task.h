/**
  ******************************************************************************
  * @file           : Control_Task.h
  * @brief          : 控制任务头文件
  * @author         : Yan Yuanbin
  * @date           : 2023/04/27
  * @version        : v1.0
  ******************************************************************************
  * @attention      : None
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef CONTROL_TASK_H
#define CONTROL_TASK_H

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "stdbool.h"


/**
 * @brief 底盘控制信息结构体。
*/

typedef struct 
{
	
	struct{
    /* 目标量 */
		float Chassis_Velocity;
		float Chassis_AngularVelocity;
	
	}Target;	
 
 struct{
    /* 反馈量 */
	 float Chassis_Velocity;
	 float Chassis_AngularVelocity;
	
	}Measure;
	
  /* 发送给执行机构的控制量 */
	int16_t SendValue[4];
	
}Control_Info_Typedef;


extern Control_Info_Typedef Control_Info;

/**
 * @brief 控制任务入口函数。
 * @param argument RTOS 线程参数，未使用。
 */
void Control_Task(void const * argument);

#endif //CONTROL_TASK_H