//
// Created by LIMBO on 2026/4/21.
//
#include "graysensor.h"
#include "grayscale.h"
#include "message_center.h"
#include "cmsis_os.h"
#include "bsp_log.h"

// 消息发布订阅
static Publisher_t *Graysensor_Pub;    // 发布灰度传感器的数据
static Subscriber_t *Graysensor_Sub;   // 订阅灰度传感器的控制命令

static Graysensor_Ctrl_Cmd_s Graysensor_Cmd_Recv;        // 接收到的控制命令
static Graysensor_Upload_Data_s Graysensor_Feedback_Data; // 反馈数据

static GrayscaleData_t *grayscale_data = NULL; // 灰度传感器数据指针

/**
 * @brief 灰度传感器应用初始化
 * 请在开启RTOS之前调用
 */
void GraysensorInit()
{
    // 初始化灰度传感器 (使用USART8)
    extern UART_HandleTypeDef huart8;
    grayscale_data = GrayscaleInit(&huart8);

    // 注册消息发布订阅
    Graysensor_Sub = SubRegister("Graysensor_Cmd", sizeof(Graysensor_Ctrl_Cmd_s));
    Graysensor_Pub = PubRegister("Graysensor_Feed", sizeof(Graysensor_Upload_Data_s));

    // 发送默认控制命令 (模拟模式)
    GrayscaleSendControl(0, 1, 0);
}

/**
 * @brief 灰度传感器应用任务
 * 放入实时系统以一定频率运行
 */
void GraysensorTask()
{
    // 获取控制命令
    SubGetMessage(Graysensor_Sub, &Graysensor_Cmd_Recv);

    // 根据控制命令设置传感器模式
    // GrayscaleSendControl(Graysensor_Cmd_Recv.calib_mode,
    //                     Graysensor_Cmd_Recv.analog_mode,
    //                     Graysensor_Cmd_Recv.digital_mode);

    // 检查是否有新数据包
    if (grayscale_data && grayscale_data->new_package_flag)
    {
        // 填充反馈数据
        for (int i = 0; i < 8; i++)
        {
            Graysensor_Feedback_Data.sensor_values[i] = grayscale_data->sensor_values[i];
        }
        Graysensor_Feedback_Data.data_valid = grayscale_data->valid;
        Graysensor_Feedback_Data.sensor_online = GrayscaleIsOnline();

        // 清除标志位
        GrayscaleClearNewPackageFlag();
    }

    // 发布反馈数据
    PubPushMessage(Graysensor_Pub, (void *)&Graysensor_Feedback_Data);
}
