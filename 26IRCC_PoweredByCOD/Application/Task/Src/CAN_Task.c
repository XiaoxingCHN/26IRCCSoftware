/**
 ******************************************************************************
 * @file           : CAN_Task.c
 * @brief          : CAN task
 * @author         : GrassFam Wang
 * @date           : 2025/1/22
 * @version        : v1.1
 ******************************************************************************
 * @attention      : None
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "cmsis_os.h"
#include "CAN_Task.h"
#include "Control_Task.h"
#include "INS_Task.h"
#include "Motor.h"
#include "bsp_can.h"
#include "Remote_Control.h"
#include "Control_Task.h"

/* USER CODE BEGIN Header_CAN_Task */
/**
 * @brief Function implementing the StartCANTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_CAN_Task */

/**
 * @brief CAN 通信任务。
 * @details
 * 启动后先完成 DM 电机使能，随后以 1kHz 周期向 CAN-FD 与
 * 标准 CAN 总线发送控制帧。
 * @param argument RTOS 线程参数，未使用。
 */
void CAN_Task(void const *argument)
{

  TickType_t CAN_Task_SysTick = 0;

  /* 分时使能 DM 电机，避免上电瞬间总线/电源冲击。 */
  // DM_Motor_Command(&FDCAN2_TxFrame, &DM_8009_Motor[0], Motor_Enable);
  // osDelay(30);
  // DM_Motor_Command(&FDCAN2_TxFrame, &DM_8009_Motor[1], Motor_Enable);
  // osDelay(30);
  // DM_Motor_Command(&FDCAN2_TxFrame, &DM_8009_Motor[2], Motor_Enable);
  // osDelay(30);
  // DM_Motor_Command(&FDCAN2_TxFrame, &DM_8009_Motor[3], Motor_Enable);
  // osDelay(30);

  /* DJI 底盘四电机群组控制 ID(0x200) */
  FDCAN1_TxFrame.Header.Identifier = Chassis_Motor[0].FDCANFrame.TxIdentifier;

  for (;;)
  {

    CAN_Task_SysTick = osKernelSysTick();

    // CAN-FD 指令更新
    // DM_Motor_CAN_TxMessage(&FDCAN2_TxFrame, &DM_8009_Motor[0], 0, 0, 0, 0, 0);
    // DM_Motor_CAN_TxMessage(&FDCAN2_TxFrame, &DM_8009_Motor[1], 0, 0, 0, 0, 0);
    // DM_Motor_CAN_TxMessage(&FDCAN2_TxFrame, &DM_8009_Motor[2], 0, 0, 0, 0, 0);
    // DM_Motor_CAN_TxMessage(&FDCAN2_TxFrame, &DM_8009_Motor[3], 0, 0, 0, 0, 0);

    /* 底盘电机控制量打包到标准 CAN 帧(大端 int16)。 */
    // FDCAN1_TxFrame.Header.Identifier = Chassis_Motor[0].FDCANFrame.TxIdentifier;
    FDCAN1_TxFrame.Data[0] = (uint8_t)(Control_Info.SendValue[0] >> 8);
    FDCAN1_TxFrame.Data[1] = (uint8_t)(Control_Info.SendValue[0]);
    FDCAN1_TxFrame.Data[2] = (uint8_t)(Control_Info.SendValue[1] >> 8);
    FDCAN1_TxFrame.Data[3] = (uint8_t)(Control_Info.SendValue[1]);
    USER_FDCAN_AddMessageToTxFifoQ(&FDCAN1_TxFrame);

    if (CAN_Task_SysTick % 2 == 0)
    {

      // 500Hz 分支，预留给后续低频 CAN 业务。
    }
    osDelay(1);
  }
}
