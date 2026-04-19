/**
 * @file chassis.c
 * @author NeoZeng neozng1@hnu.edu.cn
 * @brief 底盘应用,负责接收robot_cmd的控制命令并根据命令进行运动学解算,得到输出
 *        注意底盘采取右手系,对于平面视图,底盘纵向运动的正前方为x正方向;横向运动的右侧为y正方向
 *
 * @version 0.1
 * @date 2022-12-04
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "chassis.h"
#include "robot_def.h"
#include "dji_motor.h"
#include "super_cap.h"
#include "message_center.h"
#include "referee_task.h"

#include "general_def.h"
#include "bsp_dwt.h"
#include "referee_UI.h"
#include "arm_math.h"


static Publisher_t *Chassis_Pub;//发布底盘的数据
static Subscriber_t *Chassis_Sub;//用于订阅底盘的控制命令

static  Chassis_Ctrl_Cmd_s Chassis_Cmd_Recv;//底盘接收到的控制命令
static  Chassis_Upload_Data_s Chassis_Feedback_Data;//底盘回传的反馈数据

static DJIMotorInstance *Motor_Lf, *Motor_Lb, *Motor_Rf, *Motor_Rb;//左前【1】，左后【4】，右前【2】，右后【3】

static float Chassis_Target_Velocity = 0,Chassis_Target_Angular_Velocity = 0;//底盘的目标线速度和角速度
static volatile float Chassis_Target_VLF = 0,Chassis_Target_VLB = 0,Chassis_Target_VRF = 0,Chassis_Target_VRB=0;//每个轮子的目标速度

void ChassisInit() {
    //四个轮子的参数是一样的，只是ID和转速有差别从后往前看顺时针左上为ID1
    Motor_Init_Config_s Chassis_Motor_config ={
        .can_init_config = &hcan1, // 修改为对应的CAN接口
        .controller_param_init_config = {
            .speed_PID = {
                .Kp = 60.5f,
                .Ki = .0f,
                .Kd = .0f,
                .IntegralLimit = 3000,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut = 15000,
                .Output_LPF_RC = 0.3f,

            },
        },
        .controller_setting_init_config = {
        .angle_feedback_source = MOTOR_FEED,
        .speed_feedback_source = MOTOR_FEED,
        .outer_loop_type = SPEED_LOOP,
        .close_loop_type = SPEED_LOOP,
        },
        .motor_type = M3508,
    };
    Chassis_Motor_config.can_init_config.tx_id = 1;//左前
    Chassis_Motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    Motor_Lf = DJIMotorInit(&Chassis_Motor_config);

    Chassis_Motor_config.can_init_config.tx_id = 2;//右前
    Chassis_Motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    Motor_Rf = DJIMotorInit(&Chassis_Motor_config);

    Chassis_Motor_config.can_init_config.tx_id = 3;//右后
    Chassis_Motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    Motor_Rb = DJIMotorInit(&Chassis_Motor_config);

    Chassis_Motor_config.can_init_config.tx_id = 4;//左后
    Chassis_Motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    Motor_Lb = DJIMotorInit(&Chassis_Motor_config);

    Chassis_Sub = SubRegister("Chassis_Cmd",sizeof(Chassis_Ctrl_Cmd_s));
    Chassis_Pub = PubRegister("Chassis_Feed",sizeof(Chassis_Upload_Data_s));

}
static void MecanumCalculate()
{

    Chassis_Target_VLF = Chassis_Target_Velocity - Chassis_Target_Angular_Velocity * (TRACK_WIDTH/2.0f);
    Chassis_Target_VLB = Chassis_Target_Velocity - Chassis_Target_Angular_Velocity * (TRACK_WIDTH/2.0f);
    Chassis_Target_VRF = Chassis_Target_Velocity + Chassis_Target_Angular_Velocity * (TRACK_WIDTH/2.0f);
    Chassis_Target_VRB = Chassis_Target_Velocity + Chassis_Target_Angular_Velocity * (TRACK_WIDTH/2.0f);

}
/* 机器人底盘控制核心任务 */
void ChassisTask()
{

    SubGetMessage(Chassis_Sub,&Chassis_Cmd_Recv);//获取底盘命令信息

    if (Chassis_Cmd_Recv.chassis_mode == CHASSIS_ZERO_FORCE) {
        //急停模式
        DJIMotorStop(Motor_Lf);
        DJIMotorStop(Motor_Lb);
        DJIMotorStop(Motor_Rf);
        DJIMotorStop(Motor_Rb);

    }
    else {
        //正常模式
        DJIMotorEnable(Motor_Lf);
        DJIMotorEnable(Motor_Lb);
        DJIMotorEnable(Motor_Rf);
        DJIMotorEnable(Motor_Rb);
    }

    Chassis_Target_Velocity=Chassis_Cmd_Recv.vx;
    Chassis_Target_Angular_Velocity=Chassis_Cmd_Recv.wz;
    MecanumCalculate();
    DJIMotorSetRef(Motor_Lb, Chassis_Target_VLB);
    DJIMotorSetRef(Motor_Lf, Chassis_Target_VLF);
    DJIMotorSetRef(Motor_Rb, Chassis_Target_VRB);
    DJIMotorSetRef(Motor_Rf, Chassis_Target_VRF);
    PubPushMessage(Chassis_Pub,(void *)&Chassis_Feedback_Data);//发布底盘反馈数据,目前还没有填充数据,后续增加
}