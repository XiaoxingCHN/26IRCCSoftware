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
static PIDInstance Yaw_Angle_Controller, Yaw_Angle_Velocity_Controller;//用于底盘跟随yaw角度的pid控制器

static float Chassis_Target_Velocity = 0,Chassis_Target_Angular_Velocity = 0;//底盘的目标线速度和角速度
static float Target_wz_offset = 0;

static volatile float Chassis_Target_VLF = 0,Chassis_Target_VLB = 0,Chassis_Target_VRF = 0,Chassis_Target_VRB=0;//每个轮子的目标速度

void ChassisInit() {
    //四个轮子的参数是一样的，只是ID和转速有差别从后往前看顺时针左上为ID1
    Motor_Init_Config_s Chassis_Motor_config ={
        .can_init_config = &hcan1, // 修改为对应的CAN接口
        .controller_param_init_config = {
            .speed_PID = {

                .Kp = 450.f,//40
                .Ki = 50.7f,
                .Kd = 5.1f,
                .IntegralLimit = 5000,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement | PID_OutputFilter,
                .MaxOut = 20000,
                .Output_LPF_RC =  0.21,
                .Derivative_LPF_RC = 0.52f,
            },
            .current_PID = {
                .Kp = 0.5f, // 0.5
                .Ki = 0.f,   // 0
                .Kd = 0.f,
                .IntegralLimit = 3000,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut = 20000,
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

    PID_Init_Config_s Yaw_Angle_Compensator_Config = {
        .Kp = 0.8,
        .Ki = 0.0,
        .Kd = 0.01,
        .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement | PID_OutputFilter,
        .IntegralLimit = 2000,
        .MaxOut = 5000,
        .DeadBand = 2,
        .Output_LPF_RC = 0.1,
    };

    PID_Init_Config_s Yaw_Angle_Velocity_Compensator_Config = {
        .Kp = 10.0,
        .Ki = 0.0,
        .Kd = 0.5,
        .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement | PID_OutputFilter,
        .IntegralLimit = 2000,
        .MaxOut = 5000,
        .DeadBand = 0,
        .Output_LPF_RC = 0.1,
    };

    PIDInit(&Yaw_Angle_Velocity_Controller, &Yaw_Angle_Velocity_Compensator_Config);
    PIDInit(&Yaw_Angle_Controller, &Yaw_Angle_Compensator_Config);

    Chassis_Sub = SubRegister("Chassis_Cmd",sizeof(Chassis_Ctrl_Cmd_s));
    Chassis_Pub = PubRegister("Chassis_Feed",sizeof(Chassis_Upload_Data_s));

}

static void MecanumCalculate()
{

    Chassis_Target_VLF = Chassis_Target_Velocity + Chassis_Target_Angular_Velocity * (TRACK_WIDTH/2.0f);
    Chassis_Target_VLB = Chassis_Target_Velocity + Chassis_Target_Angular_Velocity * (TRACK_WIDTH/2.0f);
    Chassis_Target_VRF = Chassis_Target_Velocity - Chassis_Target_Angular_Velocity * (TRACK_WIDTH/2.0f);
    Chassis_Target_VRB = Chassis_Target_Velocity - Chassis_Target_Angular_Velocity * (TRACK_WIDTH/2.0f);

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
        Chassis_Target_Velocity = 0.0f;
        Chassis_Target_Angular_Velocity = 0.0f;

        break;
    case CHASSIS_NORMAL:
        EnableAllMotor();
        Chassis_Target_Velocity = Chassis_Cmd_Recv.vx;

        if (Chassis_Cmd_Recv.wz == 0)
        {
            Target_wz_offset = PIDCalculate(&Yaw_Angle_Controller, -Chassis_Cmd_Recv.yaw_angle, -Chassis_Cmd_Recv.target_yaw_angle);
            Chassis_Target_Angular_Velocity = Target_wz_offset;
        }
        else
        {
            Target_wz_offset = 0.0f; // 清除偏移量
            Chassis_Target_Angular_Velocity = PIDCalculate(&Yaw_Angle_Velocity_Controller, -Chassis_Cmd_Recv.yaw_angle_speed, Chassis_Cmd_Recv.wz);
        }

        break;
    default:
        break;
    }
    // Temp_Target_wz = PIDCalculate(&Yaw_Angle_Controller, Chassis_Cmd_Recv.offset_angle, 0.0f);

    MecanumCalculate();
    UpdateMotorRef();

    PubPushMessage(Chassis_Pub,(void *)&Chassis_Feedback_Data);//发布底盘反馈数据,目前还没有填充数据,后续增加
}