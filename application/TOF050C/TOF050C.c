//
// Created by LIMBO on 2026/4/23.
//
#include "TOF050C.h"
#include "vl6180x.h"
#include "vl53l0.h"
#include "message_center.h"

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
    uint8_t data_valid;       // 数据是否有效
    uint8_t sensor_online;    // 传感器是否在线
} TOF050C_Upload_Data_s;

// 消息发布订阅
static Publisher_t *TOF050C_Pub;    // 发布TOF050C的数据
static Subscriber_t *TOF050C_Sub;   // 订阅TOF050C的控制命令

static TOF050C_Ctrl_Cmd_s TOF050C_Cmd_Recv;        // 接收到的控制命令
static TOF050C_Upload_Data_s TOF050C_Feedback_Data; // 反馈数据

/**
 * @brief TOF050C应用初始化
 * 请在开启RTOS之前调用
 */
void TOF050CInit()
{
    // 初始化VL6180X多传感器
    multisensor_vl6180x();

    // 初始化VL53L0多传感器
    multisensor_vl53l0_Init();

    // 注册消息发布订阅
    TOF050C_Sub = SubRegister("TOF050C_Cmd", sizeof(TOF050C_Ctrl_Cmd_s));
    TOF050C_Pub = PubRegister("TOF050C_Feed", sizeof(TOF050C_Upload_Data_s));
}

/**
 * @brief TOF050C应用任务
 * 放入实时系统以一定频率运行
 */
void TOF050CTask()
{
    // 获取控制命令
    SubGetMessage(TOF050C_Sub, &TOF050C_Cmd_Recv);

    // 读取8个传感器的测距数据，前4 TOF050C，后4 TOF200C
    for (int i = 0; i < 8; i++)
    {
        if (i < 4)
        {
            // TOF050C (VL6180X)
            uint8_t address = 0x30 + i;
            uint8_t addr_write = address << 1;
            uint8_t addr_read = (address << 1) | 0x01;
            TOF050C_Feedback_Data.range_values[i] = VL6180X_ReadRangeSingleMillimeters(addr_write, addr_read);
        }
        else
        {
            // TOF200C (VL53L0)
            uint8_t address = 0x30 +i;
            TOF050C_Feedback_Data.range_values[i] = VL53L0X_readRangeSingleMillimeters(address);
        }
    }

    // 设置数据有效性和在线状态
    TOF050C_Feedback_Data.data_valid = 1;   // 假设数据总是有效
    TOF050C_Feedback_Data.sensor_online = 1; // 假设传感器总是在线

    // 发布反馈数据
    PubPushMessage(TOF050C_Pub, (void *)&TOF050C_Feedback_Data);
}
