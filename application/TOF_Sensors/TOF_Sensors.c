//
// Created by LIMBO on 2026/5/2.
//
#include "TOF_Sensors.h"
#include "vl6180x.h"
#include "vl53l0.h"
#include "message_center.h"
#include "bsp_iic.h"
#include "stdio.h"
#include "usart.h"
extern IICInstance *tof_iic; // BSP I2C 实例（定义在 vl53l0.c）

/**
 * @brief TOF050C控制命令结构体定义
 */
typedef struct {
	// 控制命令字段，如果需要
} TOF050C_Ctrl_Cmd_s;

/**
 * @brief TOF050C上传数据结构体定义
 */
typedef struct {
	uint16_t range_values[8]; // 8个传感器的测距值，前4 TOF050C，后4 TOF200C

	uint8_t data_valid; // 数据是否有效
	uint8_t sensor_online; // 传感器是否在线
} TOF050C_Upload_Data_s;

// 消息发布订阅
static Publisher_t *TOF050C_Pub; // 发布TOF050C的数据
static Subscriber_t *TOF050C_Sub; // 订阅TOF050C的控制命令

static TOF050C_Ctrl_Cmd_s TOF050C_Cmd_Recv; // 接收到的控制命令
static TOF050C_Upload_Data_s TOF050C_Feedback_Data; // 反馈数据

static const TOF_050C_WRREG vl6180x_sensors[4] = {
	{0x60, 0x61}, // 传感器0 (I2C 0x30)
	{0x62, 0x63}, // 传感器1 (I2C 0x31)
	{0x64, 0x65}, // 传感器2 (I2C 0x32)
	{0x66, 0x67}, // 传感器3 (I2C 0x33)
};
/**
 * @brief TOF050C应用初始化
 * 请在开启RTOS之前调用
 */
void TOF050CInit() {
	// 注册 TOF 共享 BSP I2C 实例（超时 100ms，替代原先的 HAL 0xffff 超时）
	IIC_Init_Config_s tof_i2c_conf = {
		.handle = &hi2c2,
		.dev_address = 0x29, // 默认地址，后续通过 IICSetDeviceAddress 切换
		.work_mode = IIC_BLOCK_MODE,
		.callback = NULL,
		.id = NULL,
	};
	tof_iic = IICRegister(&tof_i2c_conf);

	// 初始化VL6180X多传感器（TOF050C）
	multisensor_vl6180x();

	// 初始化VL53L0多传感器
	multisensor_vl53l0_Init();

	// 注册消息发布订阅
	TOF050C_Sub = SubRegister("TOF050C_Cmd", sizeof(TOF050C_Ctrl_Cmd_s));
	TOF050C_Pub = PubRegister("TOF050C_Feed", sizeof(TOF050C_Upload_Data_s));
}

// /**
//  * @brief TOF050C应用任务
//  * 放入实时系统以一定频率运行
//  */
void TOF050CTask() {
	// 获取控制命令
	SubGetMessage(TOF050C_Sub, &TOF050C_Cmd_Recv);

	// 读取8个传感器的测距数据，前4 TOF050C，后4 TOF200C
	// TOF050C (VL6180X) 传感器 0-3
	VL6180X_ReadRangeMultiple(vl6180x_sensors, TOF050C_Feedback_Data.range_values, 4);
	// TOF050C_Feedback_Data.range_values[0] = VL6180X_ReadRangeSingleMillimeters(0x60, 0x61); // 地址 0x30
	//    TOF050C_Feedback_Data.range_values[1] = VL6180X_ReadRangeSingleMillimeters(0x62, 0x63); // 地址 0x31
	//    TOF050C_Feedback_Data.range_values[2] = VL6180X_ReadRangeSingleMillimeters(0x64, 0x65); // 地址 0x32
	//    TOF050C_Feedback_Data.range_values[3] = VL6180X_ReadRangeSingleMillimeters(0x66, 0x67); // 地址 0x33
	//两侧从TOF200C改为TOF050C
	// TOF050C_Feedback_Data.range_values[4] = VL6180X_ReadRangeSingleMillimeters(0x68, 0x69); // 地址 0x34
	// TOF050C_Feedback_Data.range_values[5] = VL6180X_ReadRangeSingleMillimeters(0x6A, 0x6B); // 地址 0x35
	// TOF200C (VL53L0) 传感器 4-7
	TOF050C_Feedback_Data.range_values[4] = VL53L0X_readRangeSingleMillimeters(0x34); // 地址 0x34
	TOF050C_Feedback_Data.range_values[5] = VL53L0X_readRangeSingleMillimeters(0x35); // 地址 0x35
	// TOF050C_Feedback_Data.range_values[6] = VL53L0X_readRangeSingleMillimeters(0x36); // 地址 0x36
	// TOF050C_Feedback_Data.range_values[7] = VL53L0X_readRangeSingleMillimeters(0x37); // 地址 0x37

	char tx_buf[128];
	int len = sprintf(tx_buf, "%d,%d,%d,%d,%d,%d\r\n",
	                  TOF050C_Feedback_Data.range_values[0],
	                  TOF050C_Feedback_Data.range_values[1],
	                  TOF050C_Feedback_Data.range_values[2],
	                  TOF050C_Feedback_Data.range_values[3],
	                  TOF050C_Feedback_Data.range_values[4],
	                  TOF050C_Feedback_Data.range_values[5]
	);

	HAL_UART_Transmit(&huart8, (uint8_t *) tx_buf, len, 100);

	// 设置数据有效性和在线状态
	TOF050C_Feedback_Data.data_valid = 1; // 假设数据总是有效
	TOF050C_Feedback_Data.sensor_online = 1; // 假设传感器总是在线

	// 发布反馈数据
	PubPushMessage(TOF050C_Pub, (void *) &TOF050C_Feedback_Data);
}
