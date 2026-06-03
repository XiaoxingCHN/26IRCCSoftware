/* 注意该文件应只用于任务初始化,只能被robot.c包含*/
#pragma once

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

#include "robot.h"
#include "ins_task.h"
#include "motor_task.h"
#include "referee_task.h"
#include "master_process.h"
#include "daemon.h"
#include "HT04.h"
#include "buzzer.h"

#include "bsp_log.h"
#include "graysensor.h"
#include "TOF_Sensors.h"
#include "bsp_adc.h"
#include "buzzer.h"

osThreadId insTaskHandle;
osThreadId robotTaskHandle;
osThreadId motorTaskHandle;
osThreadId daemonTaskHandle;
osThreadId uiTaskHandle;
// osThreadId tof050cTaskHandle;

void StartINSTASK(void const *argument);

void StartMOTORTASK(void const *argument);

void StartDAEMONTASK(void const *argument);

void StartROBOTTASK(void const *argument);

void StartUITASK(void const *argument);

// void StartTOF050CTASK(void const *argument);
/**
 * @brief 初始化机器人任务,所有持续运行的任务都在这里初始化
 *
 */
void OSTaskInit() {
	osThreadDef(instask, StartINSTASK, osPriorityAboveNormal, 0, 1024);
	insTaskHandle = osThreadCreate(osThread(instask), NULL); // 由于是阻塞读取传感器,为姿态解算设置较高优先级,确保以1khz的频率执行
	// // 后续修改为读取传感器数据准备好的中断处理,

	osThreadDef(motortask, StartMOTORTASK, osPriorityNormal, 0, 256);
	motorTaskHandle = osThreadCreate(osThread(motortask), NULL);

	osThreadDef(daemontask, StartDAEMONTASK, osPriorityNormal, 0, 128);
	daemonTaskHandle = osThreadCreate(osThread(daemontask), NULL);

	osThreadDef(robottask, StartROBOTTASK, osPriorityNormal, 0, 1024);
	robotTaskHandle = osThreadCreate(osThread(robottask), NULL);

	osThreadDef(uitask, StartUITASK, osPriorityNormal, 0, 512);
	uiTaskHandle = osThreadCreate(osThread(uitask), NULL);

	// osThreadDef(tof050ctask, StartTOF050CTASK, osPriorityNormal, 0, 256);
	// tof050cTaskHandle = osThreadCreate(osThread(tof050ctask), NULL);

	HTMotorControlInit(); // 没有注册HT电机则不会执行
}

__attribute__((noreturn)) void StartINSTASK(void const *argument) {
	static float ins_start;
	static float ins_dt;
	INS_Init(); // 确保BMI088被正确初始化.0
	//   static BuzzzerInstance *robocmd_alarm;
	//   Buzzer_config_s buzzer_config ={
	//     .alarm_level = ALARM_LEVEL_HIGH, //设置警报等级 同一状态下 高等级的响应
	//     .loudness=  0.4, //设置响度
	//     .octave=  OCTAVE_1, // 设置音阶
	// };
	// robocmd_alarm = BuzzerRegister(&buzzer_config);
	// AlarmSetStatus(robocmd_alarm, ALARM_ON);
	LOGINFO("[freeRTOS] INS Task Start");
	for (;;) {
		// 1kHz
		ins_start = DWT_GetTimeline_ms();
		INS_Task();
		ins_dt = DWT_GetTimeline_ms() - ins_start;
		// if (ins_dt >2) AlarmSetStatus(robocmd_alarm, ALARM_OFF);
		if (ins_dt > 1)
			LOGERROR("[freeRTOS] INS Task is being DELAY! dt = [%f]", &ins_dt);
		VisionSend(); // 解算完成后发送视觉数据,但是当前的实现不太优雅,后续若添加硬件触发需要重新考虑结构的组织
		osDelay(1);
	}
}

__attribute__((noreturn)) void StartMOTORTASK(void const *argument) {
	static float motor_dt;
	static float motor_start;
	LOGINFO("[freeRTOS] MOTOR Task Start");
	for (;;) {
		motor_start = DWT_GetTimeline_ms();
		MotorControlTask();
		motor_dt = DWT_GetTimeline_ms() - motor_start;
		if (motor_dt > 1)
			LOGERROR("[freeRTOS] MOTOR Task is being DELAY! dt = [%f]", &motor_dt);
		osDelay(1);
	}
}

__attribute__((noreturn)) void StartDAEMONTASK(void const *argument) {
	static float daemon_dt;
	static float daemon_start;
	BuzzerInit();
	LOGINFO("[freeRTOS] Daemon Task Start");
	for (;;) {
		// 100Hz
		daemon_start = DWT_GetTimeline_ms();
		DaemonTask();
		BuzzerTask();
		daemon_dt = DWT_GetTimeline_ms() - daemon_start;
		if (daemon_dt > 10)
			LOGERROR("[freeRTOS] Daemon Task is being DELAY! dt = [%f]", &daemon_dt);
		osDelay(10);
	}
}

__attribute__((noreturn)) void StartROBOTTASK(void const *argument) {
	static float robot_dt;
	static float robot_start;
	LOGINFO("[freeRTOS] ROBOT core Task Start");
	// 200Hz-500Hz,若有额外的控制任务如平衡步兵可能需要提升至1kHz
	for (;;) {
		robot_start = DWT_GetTimeline_ms();
		RobotTask();
		robot_dt = DWT_GetTimeline_ms() - robot_start;
		if (robot_dt > 5)
			LOGERROR("[freeRTOS] ROBOT core Task is being DELAY! dt = [%f]", &robot_dt);
		osDelay(5);
	}
}

__attribute__((noreturn)) void StartUITASK(void const *argument) {
	LOGINFO("[freeRTOS] UI Task Start");
	// MyUIInit();
	static float VBAT_Monitor; //电池电压监控值
	static float VBAT_Alarm_dt;
	static float VBAT_Alarm_start;
	static bool alarmflag = false;
	static Buzzer_config_s buzzer_config = {
		.alarm_level = ALARM_LEVEL_HIGH, //设置警报等级 同一状态下 高等级的响应
		.loudness = 0.9, //设置响度
		.octave = OCTAVE_3, // 设置音阶
	};
	BuzzzerInstance *robocmd_alarm = BuzzerRegister(&buzzer_config);
	// GraysensorInit(); // 初始化灰度传感器
	ADC_VBAT_INIT(); // 初始化电压检测
	TOF050CInit();

	LOGINFO("[freeRTOS] UI Init Done, communication with ref has established");
	for (;;) {
		// GraysensorTask(); // 灰度传感器任务

		TOF050CTask();
		BuzzerTask();
		// 每给裁判系统发送一包数据会挂起一次,详见UITask函数的refereeSend()
		// UITask();
		VBAT_Monitor = VBATVAL_GET();
		// Printf_UART("VBAT: %dV\r\n",(int8_t)VBAT_Monitor);
		if (VBAT_Monitor > 18.f) {
			AlarmSetStatus(robocmd_alarm, ALARM_OFF);
			VBAT_Alarm_start = DWT_GetTimeline_ms();
			alarmflag = false;
		}
		else {
			VBAT_Alarm_dt = DWT_GetTimeline_ms() - VBAT_Alarm_start;
			if (VBAT_Alarm_dt > 50) {
				if (!alarmflag) {
					AlarmSetStatus(robocmd_alarm, ALARM_ON);
					alarmflag = true;
					VBAT_Alarm_start = DWT_GetTimeline_ms();
				}
				else {
					AlarmSetStatus(robocmd_alarm, ALARM_OFF);
					alarmflag = false;
				}
			}
		}
		osDelay(1); // 即使没有任何UI需要刷新,也挂起一次,防止卡在UITask中无法切换
	}
}
