// app
#include "robot_def.h"
#include "robot_cmd.h"
// module
#include "remote_control.h"
#include "ins_task.h"
#include "master_process.h"
#include "message_center.h"
#include "general_def.h"
#include "dji_motor.h"
#include "bmi088.h"

// bsp
#include "bsp_dwt.h"
#include "bsp_log.h"
#include "bsp_usb.h"
// 私有宏,自动将编码器转换成角度值
#define YAW_ALIGN_ANGLE (YAW_CHASSIS_ALIGN_ECD * ECD_ANGLE_COEF_DJI) // 对齐时的角度,0-360
#define PTICH_HORIZON_ANGLE (PITCH_HORIZON_ECD * ECD_ANGLE_COEF_DJI) // pitch水平时电机的角度,0-360

static Publisher_t *Chassis_Cmd_Pub;//底盘控制消息发布者
static Subscriber_t *Chassis_Feed_Sub;//底盘反馈信息订阅者

static Chassis_Ctrl_Cmd_s Chassis_Cmd_Send; //发送给底盘应用的信息，包括控制信息
static Chassis_Upload_Data_s Chassis_Fetch_Data;//从底盘应用接收的反馈信息与底盘的运动状态

static RC_ctrl_t *rc_data;              // 遥控器数据,初始化时返回
static Vision_Recv_s *vision_recv_data; // 视觉接收数据指针,初始化时返回
static Vision_Send_s vision_send_data;  // 视觉发送数据

static Robot_Status_e Robot_State;//机器人整体工作状态

BMI088Instance *bmi088_test; // 云台IMU
BMI088_Data_t bmi088_data;
void RobotCMDInit()
{
    rc_data = RemoteControlInit(&huart5);   // 修改为对应串口,注意如果是自研板dbus协议串口需选用添加了反相器的那个
    vision_recv_data = VisionInit(&huart9); // 视觉通信串口

    Chassis_Cmd_Pub = PubRegister("Chassis_Cmd",sizeof(Chassis_Ctrl_Cmd_s));
    Chassis_Feed_Sub = SubRegister("Chassis_Feed",sizeof(Chassis_Upload_Data_s));

    Robot_State
    =ROBOT_READY;

}

/**
 * @brief 控制输入为遥控器(调试时)的模式和控制量设置
 */
static void RemoteControlSet(void)
{
    if (switch_is_down(rc_data[TEMP].rc.switch_left)) {
        Chassis_Cmd_Send.chassis_mode =CHASSIS_ZERO_FORCE;
    }

    else if (switch_is_up(rc_data[TEMP].rc.switch_left)) {
        Chassis_Cmd_Send.chassis_mode = CHASSIS_NORMAL;
    }
    //差速底盘只有角速度和Vx
    Chassis_Cmd_Send.vx = 2*rc_data[TEMP].rc.rocker_l1;//左拨杆竖直方向，控制前后速度
    Chassis_Cmd_Send.wz = 0.5*rc_data[TEMP].rc.rocker_r_;//右拨杆左右方向，控制左右速度
}
/* 机器人核心控制任务,200Hz频率运行(必须高于视觉发送频率) */
void RobotCMDTask()
{
    // USBTransmit("This is RobotCMDTask\r\n", 20); // 调试用,后续删除
    VisionSend(&vision_send_data);
    RemoteControlSet();
    SubGetMessage(Chassis_Feed_Sub, (void *)&Chassis_Fetch_Data);
    PubPushMessage(Chassis_Cmd_Pub, (void *)&Chassis_Cmd_Send);

}


