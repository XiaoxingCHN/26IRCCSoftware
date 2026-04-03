/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : INS_Task.c
  * @brief          : INS task
  * @author         : GrassFan Wang
  * @date           : 2025/01/22
  * @version        : v1.0
  ******************************************************************************
  * @attention      : None
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "cmsis_os.h"
#include "INS_Task.h"
#include "bmi088.h"
#include "lpf.h"
#include "pid.h"
#include "config.h"
#include "tim.h"
#include "Quaternion.h"
#include "bsp_pwm.h"

/**
  * @brief INS 全局状态量，供其他模块读取。
  */
INS_Info_Typedef INS_Info; 

/**
  * @brief 加速度计三轴二阶低通滤波器系数。
  */
static float INS_LPF2p_Alpha[3]={1.929454039488895f, -0.93178349823448126f, 0.002329458745586203f};

/**
  * @brief 加速度计 X/Y/Z 三轴低通滤波器状态。
  */
LowPassFilter2p_Info_TypeDef INS_AccelPF2p[3];  

/**
  * @brief 四元数 EKF 的状态转移矩阵初值。
  */
static float QuaternionEKF_A_Data[36]={1, 0, 0, 0, 0, 0,
                                       0, 1, 0, 0, 0, 0,
                                       0, 0, 1, 0, 0, 0,
                                       0, 0, 0, 1, 0, 0,
                                       0, 0, 0, 0, 1, 0,
                                       0, 0, 0, 0, 0, 1};

/**
  * @brief 四元数 EKF 的后验协方差矩阵初值。
  */																
static float QuaternionEKF_P_Data[36]= {100000, 0.1, 0.1, 0.1, 0.1, 0.1,
                                        0.1, 100000, 0.1, 0.1, 0.1, 0.1,
                                        0.1, 0.1, 100000, 0.1, 0.1, 0.1,
                                        0.1, 0.1, 0.1, 100000, 0.1, 0.1,
                                        0.1, 0.1, 0.1,   0.1,  100, 0.1,
                                        0.1, 0.1, 0.1,   0.1,  0.1, 100};

/**
  * @brief BMI088 温控环 PID 参数。
  * @note  顺序: KP, KI, KD, Alpha, Deadband, I_limit, Output_limit。
  */																				
static float TemCtrl_PID_Param[PID_PARAMETER_NUM]={1200,20,0,0,0,0,2000};

/**
  * @brief 温控 PID 运行时实例。
  */
PID_Info_TypeDef TempCtrl_PID;

/**
 * @brief 初始化 INS 任务相关滤波器、EKF 与温控模块。
 */
static void INS_Task_Init(void);

/**
  * @brief BMI088 加热功率闭环控制。
  * @param Temp BMI088 当前温度(摄氏度)。
  */
static void BMI088_Temp_Control(float temp);

/* USER CODE BEGIN Header_INS_Task */
/**
  * @brief  Function implementing the StartINSTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_INS_Task */
void INS_Task(void const * argument)
{
  /* USER CODE BEGIN INS_Task */
  TickType_t INS_Task_SysTick = 0;
	float plustick=0,correction=0;

	/* Initializes the INS_Task. */
	INS_Task_Init();
	
  /* Infinite loop */
  for(;;)
  {
    /* 记录当前唤醒时刻，用于周期调度与分频执行。 */
    INS_Task_SysTick = osKernelSysTick();
		
    /* 更新 BMI088 原始测量值 */
    BMI088_Info_Update(&BMI088_Info);

    /* 对加速度计三轴做二阶低通滤波 */
    INS_Info.Accel[0]   =   LowPassFilter2p_Update(&INS_AccelPF2p[0],BMI088_Info.Accel[0]);
    INS_Info.Accel[1]   =   LowPassFilter2p_Update(&INS_AccelPF2p[1],BMI088_Info.Accel[1]);
    INS_Info.Accel[2]   =   LowPassFilter2p_Update(&INS_AccelPF2p[2],BMI088_Info.Accel[2]);
		
    /* 更新 INS 角速度(弧度制) */
		INS_Info.Gyro[0]   =   BMI088_Info.Gyro[0];
    INS_Info.Gyro[1]   =   BMI088_Info.Gyro[1];
    INS_Info.Gyro[2]   =   BMI088_Info.Gyro[2];
		
    /* 更新四元数 EKF */
    QuaternionEKF_Update(&Quaternion_Info,INS_Info.Gyro,INS_Info.Accel,0.001f);
		
    /* 保留欧拉角弧度值给内部算法使用 */
    memcpy(INS_Info.Angle,Quaternion_Info.EulerAngle,sizeof(INS_Info.Angle));

    /* 更新欧拉角(角度制) */
    INS_Info.Pitch_Angle = Quaternion_Info.EulerAngle[IMU_ANGLE_INDEX_PITCH]*57.295779513f;
    INS_Info.Yaw_Angle   = Quaternion_Info.EulerAngle[IMU_ANGLE_INDEX_YAW]   *57.295779513f-correction;
    INS_Info.Roll_Angle  = Quaternion_Info.EulerAngle[IMU_ANGLE_INDEX_ROLL]*57.295779513f;
		
    /* 更新航向累计角，处理跨 +/-180 度回环 */
		if(INS_Info.Yaw_Angle - INS_Info.Last_Yaw_Angle < -180.f)
		{
			INS_Info.YawRoundCount++;
		}
		else if(INS_Info.Yaw_Angle - INS_Info.Last_Yaw_Angle > 180.f)
		{
			INS_Info.YawRoundCount--;
		}
		INS_Info.Last_Yaw_Angle = INS_Info.Yaw_Angle;
		
		INS_Info.Yaw_TolAngle = INS_Info.Yaw_Angle + INS_Info.YawRoundCount*360.f;
		
    /* 更新角速度(角度制) */
    INS_Info.Pitch_Gyro = INS_Info.Gyro[IMU_GYRO_INDEX_PITCH]*RadiansToDegrees;
    INS_Info.Yaw_Gyro   = INS_Info.Gyro[IMU_GYRO_INDEX_YAW]*RadiansToDegrees;
    INS_Info.Roll_Gyro  = INS_Info.Gyro[IMU_GYRO_INDEX_ROLL]*RadiansToDegrees;
		
		if(INS_Task_SysTick%5 == 0)
		{
      /* INS 主循环 1kHz，温控环按 200Hz 执行。 */
			BMI088_Temp_Control(BMI088_Info.Temperature);
			
			correction= plustick*(1.0/60000);
      plustick+=5;
		}

    osDelayUntil(&INS_Task_SysTick,1);
		
  }
  /* USER CODE END INS_Task */
}
//------------------------------------------------------------------------------
/**
 * @brief 初始化 INS 相关算法模块。
 */
static void INS_Task_Init(void)
{
  /* 初始化二阶低通滤波器 */
  LowPassFilter2p_Init(&INS_AccelPF2p[0],INS_LPF2p_Alpha);
  LowPassFilter2p_Init(&INS_AccelPF2p[1],INS_LPF2p_Alpha);
  LowPassFilter2p_Init(&INS_AccelPF2p[2],INS_LPF2p_Alpha);
	
  /* 初始化温控 PID */
	PID_Init(&TempCtrl_PID,PID_POSITION,TemCtrl_PID_Param);
	
  /* 初始化四元数 EKF */
	QuaternionEKF_Init(&Quaternion_Info,10.f, 0.001f, 1000000.f,QuaternionEKF_A_Data,QuaternionEKF_P_Data);
}
//------------------------------------------------------------------------------
/**
  * @brief  根据温度误差更新 BMI088 加热输出。
  * @param  Temp BMI088 实测温度。
  * @retval none
  */
static void BMI088_Temp_Control(float Temp)
{
	PID_Calculate_Position(&TempCtrl_PID,40.f,Temp);
	
	VAL_LIMIT(TempCtrl_PID.Output,-TempCtrl_PID.Param.LimitOutput,TempCtrl_PID.Param.LimitOutput);

	Heat_Power_Control((uint16_t)(TempCtrl_PID.Output));
}
//------------------------------------------------------------------------------


