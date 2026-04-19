#include "chassis.h"
#include "robot_def.h"
#include "dji_motor.h"
#include "super_cap.h"
#include "message_center.h"
#include "referee_task.h"
#include "algorithm/controller.h"

#include "general_def.h"
#include "bsp_dwt.h"
#include "referee_UI.h"
#include "arm_math.h"

static Publisher_t *Chassis_Pub;//发布底盘的数据
static Subscriber_t *Chassis_Sub;//用于订阅底盘的控制命令

static  Chassis_Ctrl_Cmd_s Chassis_Cmd_Recv;//底盘接收到的控制命令
static  Chassis_Upload_Data_s Chassis_Feedback_Data;//底盘回传的反馈数据

static DJIMotorInstance *Motor_Lf, *Motor_Lb, *Motor_Rf, *Motor_Rb;//左前1，左后4，右前2，右后3
static PIDInstance chassis_follow_yaw_controller;

static float Chassis_Target_Velocity = 0,Chassis_Target_Angular_Velocity = 0;//底盘的目标线速度和角速度
static volatile float Chassis_Target_VLF = 0,Chassis_Target_VLB = 0,Chassis_Target_VRF = 0,Chassis_Target_VRB=0;//每个轮子的目标速度

void ChassisInit() {
    //四个轮子的参数是一样的，只是ID和转速有差别从后往前看顺时针左上为ID1
    Motor_Init_Config_s Chassis_Motor_config ={
        .can_init_config = &hcan1, // 修改为对应的CAN接口
        .controller_param_init_config = {
            .speed_PID = {
                .Kp = 10.0f,
                .Ki = 10.0f,
                .Kd = 0.0f,
                .IntegralLimit = 5000,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement | PID_OutputFilter | PID_DerivativeFilter,
                .MaxOut = 15000,
                .Output_LPF_RC = 0.005f,
                .Derivative_LPF_RC = 1.0f,
            },
            .current_PID = {
                .Kp = 0.5, // 0.5
                .Ki = 0,   // 0
                .Kd = 0,
                .IntegralLimit = 3000,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut = 15000,
            },
        },
        .controller_setting_init_config = {
        .angle_feedback_source = MOTOR_FEED,
        .speed_feedback_source = MOTOR_FEED,
        .outer_loop_type = SPEED_LOOP,
        .close_loop_type = SPEED_LOOP | CURRENT_LOOP,
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

    PID_Init_Config_s chassis_follow_yaw_pid_config = {
        .Kp = 60,
        .Ki = 20,
        .Kd = 0.1,
        .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement | PID_OutputFilter,
        .IntegralLimit = 2000,
        .MaxOut = 5000,
        .DeadBand = 0.1,
    };

    PIDInit(&chassis_follow_yaw_controller, &chassis_follow_yaw_pid_config);

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


void UpdateMotorRef()
{
    DJIMotorSetRef(Motor_Lb, Chassis_Target_VLB);
    DJIMotorSetRef(Motor_Lf, Chassis_Target_VLF);
    DJIMotorSetRef(Motor_Rb, Chassis_Target_VRB);
    DJIMotorSetRef(Motor_Rf, Chassis_Target_VRF);
}

void DisableAllMotor()
{
    DJIMotorStop(Motor_Lf);
    DJIMotorStop(Motor_Lb);
    DJIMotorStop(Motor_Rf);
    DJIMotorStop(Motor_Rb);
}

void EnableAllMotor()
{
    DJIMotorEnable(Motor_Lf);
    DJIMotorEnable(Motor_Lb);
    DJIMotorEnable(Motor_Rf);
    DJIMotorEnable(Motor_Rb);
}

/* 机器人底盘控制核心任务 */
void ChassisTask()
{

    SubGetMessage(Chassis_Sub,&Chassis_Cmd_Recv);//获取底盘命令信息

    switch (Chassis_Cmd_Recv.chassis_mode)
    {
    case CHASSIS_ZERO_FORCE:
        DisableAllMotor();
        break;
    case CHASSIS_NORMAL:
        EnableAllMotor();
        break;
    default:
        break;
    }

    Chassis_Target_Velocity = Chassis_Cmd_Recv.vx;
    Chassis_Target_Angular_Velocity = Chassis_Cmd_Recv.wz;

    MecanumCalculate();
    UpdateMotorRef();

    PubPushMessage(Chassis_Pub,(void *)&Chassis_Feedback_Data);//发布底盘反馈数据,目前还没有填充数据,后续增加
}