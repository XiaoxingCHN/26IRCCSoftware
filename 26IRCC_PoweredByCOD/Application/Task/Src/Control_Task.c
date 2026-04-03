/**
 ******************************************************************************
 * @file           : Control_Task.c
 * @brief          : Control task
 * @author         : GrassFan Wang
 * @date           : 2025/01/22
 * @version        : v1.1
 ******************************************************************************
 * @attention      : None
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "Control_Task.h"
#include "cmsis_os.h"
#include "Control_Task.h"
#include "bsp_uart.h"
#include "Remote_Control.h"
#include "PID.h"
#include "Motor.h"
#include "arm_math.h"
#include "INS_Task.h"

float target_Yaw_Angle = 0.f; // 目标偏航角，单位度
/**
 * @brief 初始化控制模块与 PID 参数。
 */
static void Control_Init(Control_Info_Typedef *Control_Info);

/**
 * @brief 更新控制环反馈测量量。
 */
static void Control_Measure_Update(Control_Info_Typedef *Control_Info);

/**
 * @brief 根据遥控输入更新控制目标值。
 */
static void Control_Target_Update(Control_Info_Typedef *Control_Info);

/**
 * @brief 运行控制器并生成电机控制输出。
 */
static void Control_Info_Update(Control_Info_Typedef *Control_Info);

/**
 * @brief 控制任务全局状态量。
 */
Control_Info_Typedef Control_Info;

//                                  KP   KI   KD  Alpha Deadband  I_MAX   Output_MAX
static float Chassis_PID_Param0[7] = {140.f, 0.2f, 0.f, 0.f, 0.f, 5000.f, 12000.f};
static float Chassis_PID_Param1[7] = {100.f, 0.1f, 0.f, 0.f, 0.f, 5000.f, 12000.f};

static float Yaw_PID_Param[7] = {100.f, 0.1f, 0.2f, 0.f, 0.f, 5000.f, 200.f};
/**
 * @brief 底盘速度环位置式 PID。
 */
PID_Info_TypeDef Chassis_PID[2];

/**
 * @brief 航向角环位置式 PID。
 */
PID_Info_TypeDef Yaw_PID;

/**
 * @brief 控制主任务，运行频率 1kHz。
 * @param argument RTOS 线程参数，未使用。
 */
void Control_Task(void const *argument)
{
  /* USER CODE BEGIN Control_Task */
  TickType_t Control_Task_SysTick = 0;

  Control_Init(&Control_Info);
  /* Infinite loop */
  for (;;)
  {
    Control_Task_SysTick = osKernelSysTick();

    /* 1) 读取反馈 2) 更新目标 3) 计算控制输出 */
    Control_Measure_Update(&Control_Info);
    Control_Target_Update(&Control_Info);
    Control_Info_Update(&Control_Info);

    /* 调试串口输出: 当前底盘速度 */
    // USART_Vofa_Justfloat_Transmit(Control_Info.Measure.Chassis_Velocity, 0.f, 0.f);

    osDelay(1);
  }
}
/* USER CODE END Control_Task */

static void Control_Init(Control_Info_Typedef *Control_Info)
{

  (void)Control_Info;
  PID_Init(&Chassis_PID[0], PID_POSITION, Chassis_PID_Param0);
  PID_Init(&Chassis_PID[1], PID_POSITION, Chassis_PID_Param1); 
  PID_Init(&Yaw_PID, PID_POSITION, Yaw_PID_Param);
}

static void Control_Measure_Update(Control_Info_Typedef *Control_Info)
{

  Control_Info->Measure.Chassis_Velocity = (Chassis_Motor[0].Data.Velocity+Chassis_Motor[1].Data.Velocity)/2.f;//R0,L1
  Control_Info->Measure.Chassis_AngularVelocity = (Chassis_Motor[1].Data.Velocity-Chassis_Motor[0].Data.Velocity)/1.2f;



}

static void Control_Target_Update(Control_Info_Typedef *Control_Info)
{
switch(remote_ctrl.rc.s[1])
{
  case 1:
  break;
  case 2:
  Control_Info->Target.Chassis_Velocity = 0;
  Control_Info->Target.Chassis_AngularVelocity = 0;
  break;
  case 3:

  // ch[3]左-前后
  // ch[0]右-左右

  /* 将遥控器通道 3 映射为目标速度 */
  Control_Info->Target.Chassis_Velocity = remote_ctrl.rc.ch[3] * 1.f;
  /* 将遥控器通道 0 映射为目标角速度 */
  if (abs(remote_ctrl.rc.ch[0]) > 100) // 遥控器通道 0 正在给指令
  {
    Control_Info->Target.Chassis_AngularVelocity = remote_ctrl.rc.ch[0] * 0.4f;
    target_Yaw_Angle = INS_Info.Yaw_TolAngle; // 不断更新，作为松手后的锁定点
  }
  else
  {
   
    
    PID_Calculate_Position(&Yaw_PID, target_Yaw_Angle, INS_Info.Yaw_TolAngle);
    // 把控制量加上去
    // Control_Info->Target.Chassis_AngularVelocity = Yaw_PID.Output; 
    Control_Info->Target.Chassis_AngularVelocity = 0;
  }
  break;
  default:
  return;

  // Control_Info->Target.Chassis_AngularVelocity = remote_ctrl.rc.ch[0] * 5.f;
  // USART_Vofa_Justfloat_Transmit(Control_Info->Target.Chassis_Velocity, Control_Info->Target.Chassis_AngularVelocity, 0.f);
}
}

static void Control_Info_Update(Control_Info_Typedef *Control_Info)
{

  
  /* 计算左右轮目标速度 */
  Chassis_Motor[1].target_speed = (Control_Info->Target.Chassis_Velocity + Control_Info->Target.Chassis_AngularVelocity * 0.6f);
  Chassis_Motor[0].target_speed = (Control_Info->Target.Chassis_Velocity - Control_Info->Target.Chassis_AngularVelocity * 0.6f);
  USART_Vofa_Justfloat_Transmit(Chassis_Motor[0].target_speed, Chassis_Motor[1].target_speed, Control_Info->Target.Chassis_Velocity);
  /* PID 输出转换为电机控制量(int16 CAN 负载)。 */
  // PID_Calculate_Position(&Chassis_PID[0], Control_Info->Target.Chassis_Velocity, Control_Info->Measure.Chassis_Velocity);
  // PID_Calculate_Position(&Chassis_PID[1], Control_Info->Target.Chassis_Velocity, Control_Info->Measure.Chassis_Velocity);
  PID_Calculate_Position(&Chassis_PID[0], Chassis_Motor[0].target_speed, Chassis_Motor[0].Data.Velocity);
  PID_Calculate_Position(&Chassis_PID[1], -Chassis_Motor[1].target_speed, Chassis_Motor[1].Data.Velocity);
  Control_Info->SendValue[0] = (int16_t)(Chassis_PID[0].Output);
  Control_Info->SendValue[1] = (int16_t)(Chassis_PID[1].Output);
}


/**
 * @brief 五次平滑插值函数。
 * @param NowTime 当前时刻。
 * @param UseTime 总时长。
 * @return 输入有效时返回 [0, 1] 区间平滑过渡值。
 */
static float FivePower(float NowTime, float UseTime)
{

  float Time = (NowTime / UseTime);

  return 10 * powf(Time, 3) - 15 * powf(Time, 4) + 6 * powf(Time, 5);

}
