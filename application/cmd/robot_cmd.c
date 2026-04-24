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
#include "TOF050C.h"
// bsp
#include "bsp_dwt.h"
#include "bsp_log.h"
#include "bsp_usb.h"
// 私有宏,自动将编码器转换成角度值
#define YAW_ALIGN_ANGLE (YAW_CHASSIS_ALIGN_ECD * ECD_ANGLE_COEF_DJI) // 对齐时的角度,0-360
#define PTICH_HORIZON_ANGLE (PITCH_HORIZON_ECD * ECD_ANGLE_COEF_DJI) // pitch水平时电机的角度,0-360
/*消息中心*/
static Publisher_t *Chassis_Cmd_Pub;//底盘控制消息发布者
static Subscriber_t *Chassis_Feed_Sub;//底盘反馈信息订阅者

static Chassis_Ctrl_Cmd_s Chassis_Cmd_Send; //发送给底盘应用的信息，包括控制信息
static Chassis_Upload_Data_s Chassis_Fetch_Data;//从底盘应用接收的反馈信息与底盘的运动状态

// 灰度传感器消息中心
static Publisher_t *Graysensor_Cmd_Pub;    // 灰度传感器控制消息发布者
static Subscriber_t *Graysensor_Feed_Sub; // 灰度传感器反馈信息订阅者

static Graysensor_Ctrl_Cmd_s Graysensor_Cmd_Send;     // 发送给灰度传感器的控制信息
static Graysensor_Upload_Data_s Graysensor_Fetch_Data; // 从灰度传感器接收的反馈信息

// TOF050C消息中心
static Publisher_t *TOF050C_Cmd_Pub;    // TOF050C控制消息发布者
static Subscriber_t *TOF050C_Feed_Sub; // TOF050C反馈信息订阅者

static TOF050C_Ctrl_Cmd_s TOF050C_Cmd_Send;     // 发送给TOF050C的控制信息
static TOF050C_Upload_Data_s TOF050C_Fetch_Data; // 从TOF050C接收的反馈信息

static RC_ctrl_t *rc_data;              // 遥控器数据,初始化时返回
static Vision_Recv_s *vision_recv_data; // 视觉接收数据指针,初始化时返回
static Vision_Send_s vision_send_data;  // 视觉发送数据
/*消息中心↑*/
static Robot_Status_e Robot_State;//机器人整体工作状态

static attitude_t *IMU_data;

static float Target_Yaw_Angele = 0, Target_Yaw_Angular_Velocity = 0, Yaw_Offset = 0;

void RobotCMDInit()
{

    IMU_data = INS_Init();//获取陀螺仪数据指针

    rc_data = RemoteControlInit(&huart5);   // 修改为对应串口,注意如果是自研板dbus协议串口需选用添加了反相器的那个
    vision_recv_data = VisionInit(&huart9); // 视觉通信串口

    Target_Yaw_Angele=IMU_data->Yaw;

  Chassis_Cmd_Pub = PubRegister("Chassis_Cmd",sizeof(Chassis_Ctrl_Cmd_s));
    Chassis_Feed_Sub = SubRegister("Chassis_Feed",sizeof(Chassis_Upload_Data_s));

    // 注册灰度传感器消息
    Graysensor_Cmd_Pub = PubRegister("Graysensor_Cmd", sizeof(Graysensor_Ctrl_Cmd_s));
    Graysensor_Feed_Sub = SubRegister("Graysensor_Feed", sizeof(Graysensor_Upload_Data_s));

    // 注册TOF050C消息
    TOF050C_Cmd_Pub = PubRegister("TOF050C_Cmd", sizeof(TOF050C_Ctrl_Cmd_s));
    TOF050C_Feed_Sub = SubRegister("TOF050C_Feed", sizeof(TOF050C_Upload_Data_s));

    // 初始化灰度传感器控制命令 (默认模拟模式)
    Graysensor_Cmd_Send.calib_mode = 0;    // 不校准
    Graysensor_Cmd_Send.analog_mode = 1;   // 模拟模式开
    Graysensor_Cmd_Send.digital_mode = 0;  // 数字模式关


    Robot_State
    =ROBOT_READY;

}

/**
 * @brief 控制输入为遥控器(调试时)的模式和控制量设置
 */
static void RemoteControlSet(void)
{
    if (switch_is_down(rc_data[TEMP].rc.switch_left)) {
        Chassis_Cmd_Send.chassis_mode = CHASSIS_ZERO_FORCE;
        Chassis_Cmd_Send.target_yaw_angle = IMU_data->Yaw;
    }else if (switch_is_mid(rc_data[TEMP].rc.switch_left)) {
        Chassis_Cmd_Send.chassis_mode = CHASSIS_NORMAL;
    }else {
        Chassis_Cmd_Send.chassis_mode = CHASSIS_ZERO_FORCE;//避免未定义的拨杆状态出现造成控制混乱
    }

    if (switch_is_down(rc_data[TEMP].rc.switch_right)) {
        //差速底盘只有角速度和Vx
        Chassis_Cmd_Send.vx = rc_data[TEMP].rc.rocker_l1 * 20.0f;
        Chassis_Cmd_Send.wz = rc_data[TEMP].rc.rocker_r_ * 0.02f;
    }
    else if (switch_is_mid(rc_data[TEMP].rc.switch_right)) {
        Chassis_Cmd_Send.vx = rc_data[TEMP].rc.rocker_l1 * -20.0f;
        Chassis_Cmd_Send.wz = rc_data[TEMP].rc.rocker_r_ * 0.02f;
    }
    else {
        Chassis_Cmd_Send.chassis_mode = CHASSIS_ZERO_FORCE;
    }
    if (Chassis_Cmd_Send.wz != 0)
    {
        Chassis_Cmd_Send.target_yaw_angle = IMU_data->Yaw;
    }

    Chassis_Cmd_Send.yaw_angle = IMU_data->Yaw;
    while (Chassis_Cmd_Send.target_yaw_angle - IMU_data->Yaw >= 180.0f)
    {
        Chassis_Cmd_Send.target_yaw_angle -= 360.0f;
    }
    while (Chassis_Cmd_Send.target_yaw_angle - IMU_data->Yaw <= -180.0f)
    {
        Chassis_Cmd_Send.target_yaw_angle += 360.0f;
    }
    // Target_Yaw_Angele += rc_data[TEMP].rc.rocker_r_ * 0.005f;
    //
    // while (Target_Yaw_Angele - IMU_data->Yaw >= 180.0f)
    // {
    //     Target_Yaw_Angele -= 360.0f;
    // }
    // while (Target_Yaw_Angele - IMU_data->Yaw <= -180.0f)
    // {
    //     Target_Yaw_Angele += 360.0f;
    // }
    // Yaw_Offset = Target_Yaw_Angele - IMU_data->Yaw;
    // Chassis_Cmd_Send.offset_angle = Yaw_Offset;
    Chassis_Cmd_Send.yaw_angle_speed = IMU_data->Gyro[2];

}
/* 机器人核心控制任务,200Hz频率运行(必须高于视觉发送频率) */
void RobotCMDTask()
{
    VisionSend(&vision_send_data);
    RemoteControlSet();

    SubGetMessage(Chassis_Feed_Sub, (void *)&Chassis_Fetch_Data);
    PubPushMessage(Chassis_Cmd_Pub, (void *)&Chassis_Cmd_Send);

    // 处理灰度传感器消息
    SubGetMessage(Graysensor_Feed_Sub, (void *)&Graysensor_Fetch_Data);
    PubPushMessage(Graysensor_Cmd_Pub, (void *)&Graysensor_Cmd_Send);


    // 处理TOF050C消息
    SubGetMessage(TOF050C_Feed_Sub, (void *)&TOF050C_Fetch_Data);
    PubPushMessage(TOF050C_Cmd_Pub, (void *)&TOF050C_Cmd_Send);


}
