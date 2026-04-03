/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : Motor.h
  * @brief          : 电机模块头文件
  * @author         : GrassFan Wang
  * @date           : 2025/01/2
  * @version        : v1.0
  ******************************************************************************
  * @attention      : None
  ******************************************************************************
  */
/* USER CODE END Header */


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef DEVICE_MOTOR_H
#define DEVICE_MOTOR_H


/* Includes ------------------------------------------------------------------*/
#include "config.h"
#include "stm32h723xx.h"
#include "bsp_can.h"

/**
 * @brief DJI 电机类型枚举。
 */
typedef enum{
    DJI_GM6020,
    DJI_M3508,
    DJI_M2006,
    DJI_MOTOR_TYPE_NUM,
}DJI_Motor_Type_e;

/**
 * @brief 达妙电机控制模式枚举。
 */
typedef enum
{
  MIT,
	POSITION_VELOCITY,
	VELOCITY,
}DM_Motor_Control_Mode_Type_e;

/**
 * @brief 达妙电机命令枚举。
 */
typedef enum{
  Motor_Enable,
  Motor_Disable,
  Motor_Save_Zero_Position,
  DM_Motor_CMD_Type_Num,
}DM_Motor_CMD_Type_e;

/**
 * @brief 电机 CAN 帧收发 ID 信息。
 */
typedef struct
{
  uint32_t TxIdentifier;   /*!< FDCAN 发送标识符 */
  uint32_t RxIdentifier;   /*!< FDCAN 接收标识符 */

}Motor_CANFrameInfo_typedef;

/**
 * @brief DJI 电机运行数据。
 */
typedef struct 
{
  bool Initlized;   /*!< 初始化标志 */
  int16_t  Current;   /*!< 电机电流 */
  int16_t  Velocity;    /*!< 电机转速 (RPM) */
  int16_t  Encoder;   /*!< 编码器值 */
  int16_t  Last_Encoder;   /*!< 上一次编码器值 */
  float    Angle;   /*!< 电机角度 (deg) */
  uint8_t  Temperature;   /*!< 电机温度 */
	
}DJI_Motor_Data_Typedef;

/**
 * @brief 达妙电机参数范围。
 */
typedef struct 
{
  float  P_MAX;
	float  V_MAX;
	float  T_MAX;
}DM_Motor_Param_Range_Typedef;

/**
 * @brief 达妙电机运行数据。
 */
typedef struct 
{
	
  bool Initlized;    /*!< 初始化标志 */
  uint8_t  State; 	 /*!< 电机状态字 */
  uint16_t  P_int;   /*!< 位置原始值 uint16 */
	uint16_t  V_int;   /*!< 速度原始值 uint16 */
	uint16_t  T_int;   /*!< 力矩原始值 uint16 */
	float  Position;   /*!< 位置量 */
  float  Velocity;   /*!< 速度量 */
  float  Torque;     /*!< 力矩量 */
  float  Temperature_MOS;   /*!< MOS 温度 */
	float  Temperature_Rotor; /*!< 转子温度 */
  float  Angle;	
	
}DM_Motor_Data_Typedef;

/**
 * @brief DJI 电机对象信息。
 */
typedef struct
{
	DJI_Motor_Type_e Type;   /*!< 电机类型 */
  Motor_CANFrameInfo_typedef FDCANFrame;    /*!< CAN 收发配置 */
	DJI_Motor_Data_Typedef Data;   /*!< 电机运行数据 */
  float target_speed; /*!< 目标转速 */
}DJI_Motor_Info_Typedef;

/**
 * @brief 达妙电机对象信息。
 */
typedef struct
{
  
	DM_Motor_Control_Mode_Type_e	Control_Mode;
  Motor_CANFrameInfo_typedef FDCANFrame;   
	DM_Motor_Param_Range_Typedef Param_Range; 
	DM_Motor_Data_Typedef Data;   

}DM_Motor_Info_Typedef;

/**
 * @brief 达妙电机控制量。
 */
typedef struct
{
  float Position;
	float Velocity;
	float KP;
	float KD;
	float Torque;
	float Angle;
}DM_Motor_Contorl_Info_Typedef;

/* Externs ------------------------------------------------------------------*/
extern DJI_Motor_Info_Typedef DJI_Yaw_Motor,Chassis_Motor[4];

extern DM_Motor_Info_Typedef DM_8009_Motor[4];

extern DM_Motor_Contorl_Info_Typedef DM_Motor_Contorl_Info[4];

/**
 * @brief 更新 DJI 电机反馈信息。
 */
extern void DJI_Motor_Info_Update(uint32_t *Identifier, uint8_t *Rx_Buf,DJI_Motor_Info_Typedef *DJI_Motor);

/**
 * @brief 更新达妙电机反馈信息。
 */
extern void DM_Motor_Info_Update(uint32_t *Identifier,uint8_t *Rx_Buf,DM_Motor_Info_Typedef *DM_Motor);

/**
 * @brief 发送达妙电机控制命令(使能/失能/保存零点等)。
 */
extern void DM_Motor_Command(FDCAN_TxFrame_TypeDef *FDCAN_TxFrame,DM_Motor_Info_Typedef *DM_Motor,uint8_t CMD);

/**
 * @brief 发送达妙电机 CAN 控制帧。
 */
extern void DM_Motor_CAN_TxMessage(FDCAN_TxFrame_TypeDef *FDCAN_TxFrame,DM_Motor_Info_Typedef *DM_Motor,float Postion, float Velocity, float KP, float KD, float Torque);

#endif //DEVICE_MOTOR_H
