/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : bsp_can.c
  * @brief          : BSP 层 CAN/FDCAN 功能实现
  * @author         : GrassFan Wang
  * @date           : 2025/01/22
  * @version        : v1.0
  ******************************************************************************
  * @attention      : 注意使能并正确配置 FDCAN 过滤器
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "fdcan.h"
#include "bsp_can.h"
#include "Motor.h"
#include "Remote_Control.h"

/**
 * @brief FDCAN 接收帧缓存结构体(FIFO0/FIFO1)。
 */
FDCAN_RxFrame_TypeDef FDCAN_RxFIFO0Frame;
FDCAN_RxFrame_TypeDef FDCAN_RxFIFO1Frame;

/**
 * @brief FDCAN1 发送帧配置(经典 CAN)。
 */
FDCAN_TxFrame_TypeDef FDCAN1_TxFrame = {
	.hcan = &hfdcan1,
  .Header.IdType = FDCAN_STANDARD_ID, 
  .Header.TxFrameType = FDCAN_DATA_FRAME,
  .Header.DataLength = FDCAN_DLC_BYTES_8,
	.Header.ErrorStateIndicator =  FDCAN_ESI_ACTIVE,
  .Header.BitRateSwitch = FDCAN_BRS_OFF,
  .Header.FDFormat =  FDCAN_CLASSIC_CAN,           
  .Header.TxEventFifoControl =  FDCAN_NO_TX_EVENTS,  
  .Header.MessageMarker = 0,
};

/**
 * @brief FDCAN2 发送帧配置(FD CAN)。
 */
FDCAN_TxFrame_TypeDef FDCAN2_TxFrame = {
  .hcan = &hfdcan2,
  .Header.IdType = FDCAN_STANDARD_ID, 
  .Header.TxFrameType = FDCAN_DATA_FRAME,
  .Header.DataLength = FDCAN_DLC_BYTES_8,
	.Header.ErrorStateIndicator =  FDCAN_ESI_ACTIVE,
  .Header.BitRateSwitch = FDCAN_BRS_ON,
  .Header.FDFormat =  FDCAN_FD_CAN,           
  .Header.TxEventFifoControl =  FDCAN_NO_TX_EVENTS,  
  .Header.MessageMarker = 0,
};

/**
 * @brief FDCAN3 发送帧配置(经典 CAN)。
 */
FDCAN_TxFrame_TypeDef FDCAN3_TxFrame = {
  .hcan = &hfdcan3,
  .Header.IdType = FDCAN_STANDARD_ID, 
  .Header.TxFrameType = FDCAN_DATA_FRAME,
  .Header.DataLength = FDCAN_DLC_BYTES_8,
	.Header.ErrorStateIndicator =  FDCAN_ESI_ACTIVE,
  .Header.BitRateSwitch = FDCAN_BRS_OFF,
  .Header.FDFormat =  FDCAN_CLASSIC_CAN,           
  .Header.TxEventFifoControl =  FDCAN_NO_TX_EVENTS,
	.Header.MessageMarker = 0,
};

/**
  * @brief  配置并启动 FDCAN 过滤器与中断。
  * @note   FDCAN1: 经典 CAN, FIFO0; FDCAN2: FD CAN, FIFO1; FDCAN3: 经典 CAN, FIFO0。
  * @param  None
  * @retval None
  */
void BSP_FDCAN_Init(void){

  FDCAN_FilterTypeDef FDCAN1_FilterConfig;
	
	/* FDCAN1 过滤器: 标准 ID + MASK 模式 + 入 FIFO0, 0/0 表示不过滤任何 ID */
	FDCAN1_FilterConfig.IdType = FDCAN_STANDARD_ID;
  FDCAN1_FilterConfig.FilterIndex = 0;
  FDCAN1_FilterConfig.FilterType = FDCAN_FILTER_MASK;
  FDCAN1_FilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  FDCAN1_FilterConfig.FilterID1 = 0x00000000;
  FDCAN1_FilterConfig.FilterID2 = 0x00000000;
  
	/* 应用过滤器配置 */
  HAL_FDCAN_ConfigFilter(&hfdcan1, &FDCAN1_FilterConfig);
		
  /* 全局过滤: 拒绝不匹配帧与远程帧 */
  HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
 
  /* 使能 FIFO0 新消息中断 */
  HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
  
	/* 启动 FDCAN1 */
  HAL_FDCAN_Start(&hfdcan1);
 	
	
	/* FDCAN2 过滤器配置 */
	FDCAN_FilterTypeDef FDCAN2_FilterConfig;
	
  FDCAN2_FilterConfig.IdType = FDCAN_STANDARD_ID;
  FDCAN2_FilterConfig.FilterIndex = 0;
  FDCAN2_FilterConfig.FilterType = FDCAN_FILTER_MASK;
  FDCAN2_FilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
  FDCAN2_FilterConfig.FilterID1 = 0x00000000;
  FDCAN2_FilterConfig.FilterID2 = 0x00000000;
  
	HAL_FDCAN_ConfigFilter(&hfdcan2, &FDCAN2_FilterConfig);

  /* 配置全局过滤 */
  HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
  
  /* 使能 FIFO1 新消息中断 */
  HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0);

  /* 使能发送延时补偿 */
  HAL_FDCAN_EnableTxDelayCompensation(&hfdcan2);
 
  /* 配置发送延时补偿参数 */
  HAL_FDCAN_ConfigTxDelayCompensation(&hfdcan2,14,14);

	/* 启动 FDCAN2 */
  HAL_FDCAN_Start(&hfdcan2);
	
	
  /* FDCAN3 过滤器: 标准 ID + MASK 模式 + 入 FIFO0 */
  FDCAN_FilterTypeDef FDCAN3_FilterConfig;
	
	FDCAN3_FilterConfig.IdType = FDCAN_STANDARD_ID;
  FDCAN3_FilterConfig.FilterIndex = 0;
  FDCAN3_FilterConfig.FilterType = FDCAN_FILTER_MASK;
  FDCAN3_FilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  FDCAN3_FilterConfig.FilterID1 = 0x00000000; 
  FDCAN3_FilterConfig.FilterID2 = 0x00000000; 
  
	HAL_FDCAN_ConfigFilter(&hfdcan3, &FDCAN3_FilterConfig);

  HAL_FDCAN_ConfigGlobalFilter(&hfdcan3, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);

  HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);

  HAL_FDCAN_Start(&hfdcan3);
}

/**
  * @brief  向指定 FDCAN 发送 FIFO 队列加入一帧消息。
  * @param  FDCAN_TxFrame 发送帧配置与数据。
  * @retval None
  */
void USER_FDCAN_AddMessageToTxFifoQ(FDCAN_TxFrame_TypeDef *FDCAN_TxFrame){

    /* 非阻塞检查：如果发送 FIFO 已经满了，直接丢弃该帧，不阻塞 RTOS 任务。
       (若由于总线物理层故障导致 CAN_BUSY，此保护防止任务卡死) */
    if (HAL_FDCAN_GetTxFifoFreeLevel(FDCAN_TxFrame->hcan) == 0U) {
       return;
    }

    HAL_FDCAN_AddMessageToTxFifoQ(FDCAN_TxFrame->hcan,&FDCAN_TxFrame->Header,FDCAN_TxFrame->Data);
 
}

/**
  * @brief  处理 FDCAN1 FIFO0 接收数据。
	* @param  Identifier 接收帧标准 ID。
	* @param  Data 接收数据数组(8字节)。
  * @retval None
  */
static void FDCAN1_RxFifo0RxHandler(uint32_t *Identifier,uint8_t Data[8])
{
   
	DJI_Motor_Info_Update(Identifier,Data,&Chassis_Motor[0]);
  DJI_Motor_Info_Update(Identifier,Data,&Chassis_Motor[1]);
	DJI_Motor_Info_Update(Identifier,Data,&Chassis_Motor[2]);
	DJI_Motor_Info_Update(Identifier,Data,&Chassis_Motor[3]);

}

/**
  * @brief  处理 FDCAN3 FIFO0 接收数据。
	* @param  Identifier 接收帧标准 ID。
	* @param  Data 接收数据数组(8字节)。
  * @retval None
  */
static void FDCAN3_RxFifo0RxHandler(uint32_t *Identifier,uint8_t Data[8])
{



}


/**
  * @brief  处理 FDCAN2 FIFO1 接收数据。
	* @param  Identifier 接收帧标准 ID。
	* @param  Data 接收数据数组(8字节)。
  * @retval None
  */
static void FDCAN2_RxFifo1RxHandler(uint32_t *Identifier,uint8_t Data[8])
{
	
	DM_Motor_Info_Update(Identifier,Data,&DM_8009_Motor[0]);
  DM_Motor_Info_Update(Identifier,Data,&DM_8009_Motor[1]);
	DM_Motor_Info_Update(Identifier,Data,&DM_8009_Motor[2]);
	DM_Motor_Info_Update(Identifier,Data,&DM_8009_Motor[3]);
	

}

/**
  * @brief  FDCAN FIFO0 接收中断回调。
  * @param  hfdcan FDCAN 句柄。
  * @param  RxFifo0ITs FIFO0 中断标志。
  * @retval None
  */
 uint32_t pendingMsgs;
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{ 
  /* 循环读取积压的所有报文，防止因 DJI 电机 1000Hz 高频反馈导致 FIFO 溢出死锁 */
  pendingMsgs = HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0);
  while(pendingMsgs > 0)
  {
    HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &FDCAN_RxFIFO0Frame.Header, FDCAN_RxFIFO0Frame.Data);
    
    if(hfdcan == &hfdcan1){	
      FDCAN1_RxFifo0RxHandler(&FDCAN_RxFIFO0Frame.Header.Identifier, FDCAN_RxFIFO0Frame.Data);
    }
    else if(hfdcan == &hfdcan3){
      FDCAN3_RxFifo0RxHandler(&FDCAN_RxFIFO0Frame.Header.Identifier, FDCAN_RxFIFO0Frame.Data);
    }
    pendingMsgs--;
  }
}
	
/**
  * @brief  FDCAN FIFO1 接收中断回调。
  * @param  hfdcan FDCAN 句柄。
  * @param  RxFifo1ITs FIFO1 中断标志。
  * @retval None
  */
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
{ 
  /* 循环读取积压的报文 */
  while(HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO1) > 0)
  {
    HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &FDCAN_RxFIFO1Frame.Header, FDCAN_RxFIFO1Frame.Data);
    if (hfdcan == &hfdcan2) {
      FDCAN2_RxFifo1RxHandler(&FDCAN_RxFIFO1Frame.Header.Identifier, FDCAN_RxFIFO1Frame.Data);
    }
  }
}