// app
#include "robot_cmd.h"

#include "robot_def.h"
// module
#include "TOF_Sensors.h"
#include "bmi088.h"
#include "dji_motor.h"
#include "general_def.h"
#include "ins_task.h"
#include "master_process.h"
#include "message_center.h"
#include "preemptiveFSM.h"
#include "remote_control.h"
// bsp
#include "bsp_dwt.h"
#include "bsp_log.h"
#include "bsp_usb.h"
// 私有宏,自动将编码器转换成角度值
#define YAW_ALIGN_ANGLE (YAW_CHASSIS_ALIGN_ECD * ECD_ANGLE_COEF_DJI) // 对齐时的角度,0-360
#define PTICH_HORIZON_ANGLE (PITCH_HORIZON_ECD * ECD_ANGLE_COEF_DJI) // pitch水平时电机的角度,0-360
/*消息中心*/
static Publisher_t *Chassis_Cmd_Pub; // 底盘控制消息发布者
static Subscriber_t *Chassis_Feed_Sub; // 底盘反馈信息订阅者

static Chassis_Ctrl_Cmd_s Chassis_Cmd_Send; // 发送给底盘应用的信息，包括控制信息
static Chassis_Upload_Data_s Chassis_Fetch_Data; // 从底盘应用接收的反馈信息与底盘的运动状态

// 灰度传感器消息中心
static Publisher_t *Graysensor_Cmd_Pub; // 灰度传感器控制消息发布者
static Subscriber_t *Graysensor_Feed_Sub; // 灰度传感器反馈信息订阅者

static Graysensor_Ctrl_Cmd_s Graysensor_Cmd_Send; // 发送给灰度传感器的控制信息
static Graysensor_Upload_Data_s Graysensor_Fetch_Data; // 从灰度传感器接收的反馈信息

// TOF050C消息中心
static Publisher_t *TOF050C_Cmd_Pub; // TOF050C控制消息发布者
static Subscriber_t *TOF050C_Feed_Sub; // TOF050C反馈信息订阅者

static TOF050C_Ctrl_Cmd_s TOF050C_Cmd_Send; // 发送给TOF050C的控制信息
static TOF050C_Upload_Data_s TOF050C_Fetch_Data; // 从TOF050C接收的反馈信息

static RC_ctrl_t *rc_data; // 遥控器数据,初始化时返回
static Vision_Recv_s *vision_recv_data; // 视觉接收数据指针,初始化时返回
/*消息中心↑*/
static Robot_Control_Mode_e Robot_State_Mode_Global; // 由遥控器控制改变
static Robot_Status_e Robot_State; // 机器人整体工作状态

static attitude_t *IMU_data;

static float Target_Yaw_Angele = 0;

static const TOF_Flag_e tof_edge_table[16] = {
	[0x0] = DIR_NONE,
	[0x1] = DIR_BACK_RIGHT, // BR
	[0x2] = DIR_BACK_LEFT, // BL
	[0x3] = DIR_BACK, // BL+BR
	[0x4] = DIR_FRONT_RIGHT, // FR
	[0x5] = DIR_RIGHT, // FR+BR
	[0x7] = DIR_RIGHT_PLUS_BACK,
	[0x8] = DIR_FRONT_LEFT, // FL
	[0xA] = DIR_LEFT, // FL+BL
	[0xB] = DIR_LEFT_PLUS_BACK,
	[0xC] = DIR_FRONT, // FL+FR
	[0xD] = DIR_RIGHT_PLUS_FRONT,
	[0xE] = DIR_LEFT_PLUS_FRONT,
	[0xF]=DIR_ALL

};
static TOF_Flag_e tof_dir = DIR_NONE;
uint8_t tof_idx;

static AGV_Control_Cmd_s AGV_GLOBAL_CMD = {
	.target_yaw_angle = 0.0f,
	.vx = 0.0f,
	.wz = 0.0f,
	.SttartFlag = false,
}; // AGV模式下才会启用的全局模式
// AGV模式下的相关参数
static AGV_State_e AGV_State = AGV_State_Stop;
static float gray_history[8][3] = {0};
static uint8_t gray_filter_idx = 0;
float Temp_Yaw_turn = 0;
bool Back_Flag = false;
bool Front_Flag = false;
bool RIGHT_PLUS_FRONT_FLAG = false;
static uint8_t Pfsm_VIsion_last_state = 0xFF;
// 状态机
Pfsm_t StartMode_Pfsm; // 最低优先级的启动模式|优先级P10
Pfsm_t ScanPlatform_Pfsm; // 自动巡台模式倒数第二|优先级P9
Pfsm_t Follow_Vision_Pfsm;

Pfsm_t AttackAvoid_Pfsm; // 躲避袭击模式|优先级P1
Pfsm_t ReloadPlatform_Pfsm; // 掉台后自动登台|优先级P0

void StartMode_PfsmHandler(Pfsm_t *pfsm, PfsmEventId_e event);

// 其他状态机的处理函数
void ScanPlatform_PfsmHandler(Pfsm_t *pfsm, PfsmEventId_e event);

void Follow_Vision_PfsmHandler(Pfsm_t *pfsm, PfsmEventId_e event);

void AttackAvoid_PfsmHandler(Pfsm_t *pfsm, PfsmEventId_e event);

void ReloadPlatform_PfsmHandler(Pfsm_t *pfsm, PfsmEventId_e event);

void RobotCMDInit() {
	IMU_data = INS_Init(); // 获取陀螺仪数据指针
	rc_data = RemoteControlInit(&huart5); // 修改为对应串口,注意如果是自研板dbus协议串口需选用添加了反相器的那个
	vision_recv_data = VisionInit(&huart9); // 视觉通信串口

	Target_Yaw_Angele = IMU_data->Yaw;

	Chassis_Cmd_Pub = PubRegister("Chassis_Cmd", sizeof(Chassis_Ctrl_Cmd_s));
	Chassis_Feed_Sub = SubRegister("Chassis_Feed", sizeof(Chassis_Upload_Data_s));

	// 注册灰度传感器消息
	Graysensor_Cmd_Pub = PubRegister("Graysensor_Cmd", sizeof(Graysensor_Ctrl_Cmd_s));
	Graysensor_Feed_Sub = SubRegister("Graysensor_Feed", sizeof(Graysensor_Upload_Data_s));

	// 初始化灰度传感器控制命令 (默认模拟模式)
	Graysensor_Cmd_Send.calib_mode = 0; // 不校准
	Graysensor_Cmd_Send.analog_mode = 1; // 模拟模式开
	Graysensor_Cmd_Send.digital_mode = 0; // 数字模式关

	// 注册TOF050C消息
	TOF050C_Cmd_Pub = PubRegister("TOF050C_Cmd", sizeof(TOF050C_Ctrl_Cmd_s));
	TOF050C_Feed_Sub = SubRegister("TOF050C_Feed", sizeof(TOF050C_Upload_Data_s));

	// 初始化Pfsm调度器
	PfsmSched_Init();
	PfsmSched_DefaultRegister(&StartMode_Pfsm, StartMode_PfsmHandler, 10);
	PfsmSched_Register(&ScanPlatform_Pfsm, ScanPlatform_PfsmHandler, 9);
	PfsmSched_Register(&Follow_Vision_Pfsm, Follow_Vision_PfsmHandler, 8);
	PfsmSched_Register(&AttackAvoid_Pfsm, AttackAvoid_PfsmHandler, 1);
	PfsmSched_Register(&ReloadPlatform_Pfsm, ReloadPlatform_PfsmHandler, 0);
	Robot_State = ROBOT_READY;
}

static void YawAngleOFFSET(void) {
	// 对齐角度修正,使其保持在-180~180度范围内,避免PID控制器出现跨越0点时的跳变
	while (Chassis_Cmd_Send.target_yaw_angle - IMU_data->Yaw >= 180.0f) {
		Chassis_Cmd_Send.target_yaw_angle -= 360.0f;
	}
	while (Chassis_Cmd_Send.target_yaw_angle - IMU_data->Yaw <= -180.0f) {
		Chassis_Cmd_Send.target_yaw_angle += 360.0f;
	}
}

/**
 * @brief 用于在自其他模式切换到AGV模式时刷新AGV状态
 */
static void AGV_State_RESET(void) {
	Temp_Yaw_turn = 0;
	Back_Flag = false;
	Front_Flag = false;
	RIGHT_PLUS_FRONT_FLAG = false;
	Pfsm_VIsion_last_state = 0xFF;
}

/**
 * @brief 控制输入为遥控器(调试时)的模式和控制量设置
 */
static void RemoteControlSet(void) {
	if (switch_is_down(rc_data[TEMP].rc.switch_left)) {
		Chassis_Cmd_Send.chassis_mode = CHASSIS_ZERO_FORCE;
	}
	else if (switch_is_mid(rc_data[TEMP].rc.switch_left)) {
		Chassis_Cmd_Send.chassis_mode = CHASSIS_NORMAL;
		/***********************************************************/
		// REMOTE测试模式
		Robot_State_Mode_Global = CONTROL_REMOTE;
	}
	else if (switch_is_up(rc_data[TEMP].rc.switch_left)) {
		Chassis_Cmd_Send.chassis_mode = CHASSIS_NORMAL;
		/***********************************************************/
		// AGV模式
		if (Robot_State_Mode_Global != CONTROL_AGV) {
			AGV_State_RESET();
		}
		Robot_State_Mode_Global = CONTROL_AGV;
	}
}

static void VisionSend_Data(void) {
	VisionSetAltitude(IMU_data->Yaw);
	// 将TOF200C的测距数据发送给上位机作为补充
	VisionSetLaserRanging(TOF050C_Fetch_Data.range_values[4], TOF050C_Fetch_Data.range_values[5],
	                      TOF050C_Fetch_Data.range_values[6], TOF050C_Fetch_Data.range_values[7]);
}

static void Chassis_Control_Send(void) {
	switch (Chassis_Cmd_Send.chassis_mode) {
		case CHASSIS_ZERO_FORCE:
			Chassis_Cmd_Send.vx = 0;
			Chassis_Cmd_Send.wz = 0;
			Chassis_Cmd_Send.target_yaw_angle = IMU_data->Yaw; // 维持当前角度
			break;
		case CHASSIS_NORMAL:
			switch (Robot_State_Mode_Global) {
				case CONTROL_REMOTE:
					Chassis_Cmd_Send.vx = rc_data[TEMP].rc.rocker_l1 * 20.f;
					Chassis_Cmd_Send.wz = rc_data[TEMP].rc.rocker_r_ * 0.05f;
					break;
				case CONTROL_AGV:
					Chassis_Cmd_Send.vx = AGV_GLOBAL_CMD.vx;
					Chassis_Cmd_Send.wz = 0;
					Chassis_Cmd_Send.target_yaw_angle = AGV_GLOBAL_CMD.target_yaw_angle;
					break;
			}
		default:
			break;
	}
	// 同步底盘目标角度在控制命令不为0时的目标角度与IMU角度一致
	if (Chassis_Cmd_Send.wz != 0) {
		Chassis_Cmd_Send.target_yaw_angle = IMU_data->Yaw;
	}
	Chassis_Cmd_Send.yaw_angle = IMU_data->Yaw;

	YawAngleOFFSET(); // 角度修正

	Chassis_Cmd_Send.yaw_angle_speed = IMU_data->Gyro[2];
}

/* 机器人核心控制任务,200Hz频率运行(必须高于视觉发送频率) */
void RobotCMDTask() {
	// 订阅底盘消息和灰度传感器消息
	SubGetMessage(Chassis_Feed_Sub, (void *) &Chassis_Fetch_Data);
	SubGetMessage(Graysensor_Feed_Sub, (void *) &Graysensor_Fetch_Data);
	SubGetMessage(TOF050C_Feed_Sub, (void *) &TOF050C_Fetch_Data);

	/*Control Code Begin*/
	VisionSend_Data();
	VisionSend();
	RemoteControlSet();
	Chassis_Control_Send();
	if (Robot_State_Mode_Global == CONTROL_AGV) {
		PfsmSched_Run();
	}
	PubPushMessage(Chassis_Cmd_Pub, (void *) &Chassis_Cmd_Send);
	PubPushMessage(Graysensor_Cmd_Pub, (void *) &Graysensor_Cmd_Send);
	PubPushMessage(TOF050C_Cmd_Pub, (void *) &TOF050C_Cmd_Send);
}

/**
 * @brief 用于检测是否已经冲上台
 * @return false 没有冲上去
 * @return true  已经冲上去
 */
static bool UporDoenJudge(void) {
	// if ()
}
/**
 * @brief 判断相应位置的状态
 */
void TOFStatusJudge(void) {
	tof_idx = 0;
	// 0x04 0x08  0x02 0x01
	// FR01 FL02 BL00 BR03
	if (TOF050C_Fetch_Data.range_values[0] > TOF050C_EDGE)
		tof_idx |= (1 << 1);
	if (TOF050C_Fetch_Data.range_values[1] > TOF050C_EDGE)
		tof_idx |= (1 << 2);
	if (TOF050C_Fetch_Data.range_values[2] > TOF050C_EDGE)
		tof_idx |= (1 << 3);
	if (TOF050C_Fetch_Data.range_values[3] > TOF050C_EDGE)
		tof_idx |= (1 << 0);
	tof_dir = tof_edge_table[tof_idx];
}

/**
 * @brief 根据灰度传感器计算速度
 */

static void Chassis_Speed_CalculateOfGray(void) {
	gray_filter_idx = (gray_filter_idx + 1) % 3;
	for (int ch = 0; ch < 8; ch++) {
		gray_history[ch][gray_filter_idx] = Graysensor_Fetch_Data.sensor_Normalized[ch];
	}
	float filtered[8];
	for (int ch = 0; ch < 8; ch++) {
		float a = gray_history[ch][0];
		float b = gray_history[ch][1];
		float c = gray_history[ch][2];
		/* 三个数排一下取中间那个 */
		if (a > b) {
			float t = a;
			a = b;
			b = t;
		}
		if (b > c) {
			float t = b;
			b = c;
			c = t;
		}
		if (a > b) {
			float t = a;
			a = b;
			b = t;
		}
		filtered[ch] = b; /* 中值 */
	}
	float max_gray = 0;
	int max_idx = 0;
	for (int ch = 0; ch < 8; ch++) {
		if (filtered[ch] > max_gray) {
			max_gray = filtered[ch];
			max_idx = ch;
		}
	}
	/*  max_gray=0(全白) → speed_norm=1 → 全速
	 *  max_gray=0.3     → speed_norm≈0.34 → 明显减速
	 *  max_gray=0.5     → speed_norm≈0.13 → 很慢
	 *  max_gray=0.7     → speed_norm≈0.03 → 几乎停了  */
	float speed_norm = 1.0f - max_gray;
	speed_norm = speed_norm * speed_norm * speed_norm; // 三次方
	// AGV_GLOBAL_CMD.vx = AGV_APPROACH_SPEED * speed_norm;

	TOFStatusJudge();
	switch (tof_dir) {
		case DIR_BACK_RIGHT: //右后
			AGV_GLOBAL_CMD.vx = AGV_APPROACH_SPEED;
			if (!Front_Flag) Temp_Yaw_turn = IMU_data->Yaw;
			Front_Flag = true;
			Back_Flag = false;

			break;
		case DIR_BACK_LEFT: //左后
			AGV_GLOBAL_CMD.vx = AGV_APPROACH_SPEED;
			if (!Front_Flag) Temp_Yaw_turn = IMU_data->Yaw;
			Front_Flag = true;
			Back_Flag = false;

			break;
		case DIR_BACK: //后
			AGV_GLOBAL_CMD.vx = AGV_APPROACH_SPEED;
			if (!Front_Flag) Temp_Yaw_turn = IMU_data->Yaw;
			Front_Flag = true;
			Back_Flag = false;
			break;
		case DIR_FRONT_RIGHT: //右前
			if (!Back_Flag) Temp_Yaw_turn = IMU_data->Yaw;
			AGV_GLOBAL_CMD.vx = -AGV_APPROACH_SPEED;
			Back_Flag = true;
			Front_Flag = false;

			break;
		case DIR_RIGHT: //右
			AGV_GLOBAL_CMD.target_yaw_angle = IMU_data->Yaw + 90.f;
			Back_Flag = false;
			Front_Flag = false;

			break;
		case DIR_FRONT_LEFT:
			if (!Back_Flag) Temp_Yaw_turn = IMU_data->Yaw;
			AGV_GLOBAL_CMD.vx = -AGV_APPROACH_SPEED;
			Back_Flag = true;
			Front_Flag = false;

			break;
		case DIR_LEFT: //左
			AGV_GLOBAL_CMD.target_yaw_angle = IMU_data->Yaw + 90.f;
			Back_Flag = false;
			Front_Flag = false;

			break;
		case DIR_FRONT:
			if (!Back_Flag) Temp_Yaw_turn = IMU_data->Yaw;
			AGV_GLOBAL_CMD.vx = -AGV_APPROACH_SPEED;
			Back_Flag = true;
			Front_Flag = false;

			break;
		case DIR_RIGHT_PLUS_FRONT:
			AGV_GLOBAL_CMD.vx = -AGV_APPROACH_SPEED;
			if (!RIGHT_PLUS_FRONT_FLAG) Temp_Yaw_turn = IMU_data->Yaw;
			RIGHT_PLUS_FRONT_FLAG = true;
			Front_Flag = false;
			Back_Flag = false;
			break;
		case DIR_LEFT_PLUS_FRONT:
			Back_Flag = false;
			Front_Flag = false;


			break;
		case DIR_LEFT_PLUS_BACK:
			Front_Flag = false;
			Back_Flag = false;

			break;
		case DIR_RIGHT_PLUS_BACK:
			Front_Flag = false;
			Back_Flag = false;
			break;
		case DIR_ALL:
			Front_Flag = false;
			Back_Flag = false;
			RIGHT_PLUS_FRONT_FLAG = false;
			break;
		default:
			if (Back_Flag == false && Front_Flag == false && RIGHT_PLUS_FRONT_FLAG == false)
				AGV_GLOBAL_CMD.vx = AGV_APPROACH_SPEED * speed_norm;
			else if (Back_Flag == true) {
				AGV_GLOBAL_CMD.vx = -AGV_APPROACH_SPEED * speed_norm;
			}
			else if (Front_Flag == true) {
				AGV_GLOBAL_CMD.vx = AGV_APPROACH_SPEED * speed_norm;
			}
			if (AGV_GLOBAL_CMD.vx < 0.05f && AGV_GLOBAL_CMD.vx > 0)
				AGV_GLOBAL_CMD.vx = 0.05f;
			else if (AGV_GLOBAL_CMD.vx > -0.05 && AGV_GLOBAL_CMD.vx < 0)
				AGV_GLOBAL_CMD.vx = -0.05f;
			break;
	}
	if (RIGHT_PLUS_FRONT_FLAG) {
		AGV_GLOBAL_CMD.target_yaw_angle = Temp_Yaw_turn - 45.f;

		if (max_gray < 0.19) {
			tof_dir = DIR_NONE;
			AGV_GLOBAL_CMD.vx = 0;
			AGV_GLOBAL_CMD.target_yaw_angle = Temp_Yaw_turn + 100.f;
			if (abs(IMU_data->Yaw - AGV_GLOBAL_CMD.target_yaw_angle) < 20.f)
				RIGHT_PLUS_FRONT_FLAG = false;
		}
	}
	if (Back_Flag && RIGHT_PLUS_FRONT_FLAG == false) {
		if (tof_dir == DIR_FRONT_RIGHT)
			AGV_GLOBAL_CMD.target_yaw_angle = Temp_Yaw_turn - 40.f;
		if (tof_dir == DIR_FRONT_LEFT)
			AGV_GLOBAL_CMD.target_yaw_angle = Temp_Yaw_turn + 40.f;
		if (max_gray < 0.19) {
			tof_dir = DIR_NONE;
			AGV_GLOBAL_CMD.target_yaw_angle = Temp_Yaw_turn + 70.f;
			AGV_GLOBAL_CMD.vx = 0.f;
			if (abs(IMU_data->Yaw - AGV_GLOBAL_CMD.target_yaw_angle) < 20.f)
				Back_Flag = false;
		}
	}
	if (Front_Flag && RIGHT_PLUS_FRONT_FLAG == false) {
		if (tof_dir == DIR_BACK_RIGHT)
			AGV_GLOBAL_CMD.target_yaw_angle = Temp_Yaw_turn + 40.f;
		if (tof_dir == DIR_BACK_LEFT)
			AGV_GLOBAL_CMD.target_yaw_angle = Temp_Yaw_turn - 40.f;
		if (max_gray < 0.19) {
			tof_dir = DIR_NONE;
			AGV_GLOBAL_CMD.target_yaw_angle = Temp_Yaw_turn + 70.f;
			AGV_GLOBAL_CMD.vx = 0.f;
			if (abs(IMU_data->Yaw - AGV_GLOBAL_CMD.target_yaw_angle) < 20.f)
				Front_Flag = false;
		}
	}

	if (vision_recv_data->cmd_state == TRACING && tof_dir == DIR_NONE)
		PfsmSched_PostEvent(
			&Follow_Vision_Pfsm, PFSM_EVENT_FINISH_LOADPLATFORM);
	// /* 航向目标保持当前朝向 */
}

static void LoadPlatform(void) {
	if (TOF050C_Fetch_Data.range_values[4] < 40) {
	}
}
static void VisionBlockJudge(const uint8_t state) {
	TOFStatusJudge();
	if (tof_dir != DIR_NONE && state != TRACING) {
		AGV_GLOBAL_CMD.vx = 200.f;
		if (tof_dir == DIR_FRONT && (Graysensor_Fetch_Data.sensor_Normalized[4] > 0.8 || Graysensor_Fetch_Data.
					     sensor_Normalized[5] > 0.8))
			PfsmSched_Block(&Follow_Vision_Pfsm);
	}
	if (tof_dir != DIR_NONE) PfsmSched_Block(&Follow_Vision_Pfsm); // 如果TOF有障碍物,则阻塞视觉跟随状态机,等待避障完成

	if (Pfsm_VIsion_last_state != state) {
		if (Pfsm_VIsion_last_state == TRACING || Pfsm_VIsion_last_state == SEARCH_TRABLE) {
			AGV_GLOBAL_CMD.target_yaw_angle = IMU_data->Yaw; // ← 锁当前航向
		}
		Pfsm_VIsion_last_state = state;
	}

}
void StartMode_PfsmHandler(Pfsm_t *pfsm, PfsmEventId_e event) {
	uint16_t StartJudge = TOF050C_Fetch_Data.range_values[4];
	// if (StartJudge <StartFlagDistance)  AGV_State = AGV_State_Running_Auto_Down;
	// else if () {
	// 	AGV_State = AGV_State_Running_Auto_Up;
	// }
	if (switch_is_mid(rc_data[TEMP].rc.switch_right)) {
		AGV_State = AGV_State_Running_Auto_Down;
	}

	switch (AGV_State) {
		case AGV_State_Stop:
			AGV_GLOBAL_CMD.vx=0;
			AGV_GLOBAL_CMD.wz=0;
			AGV_GLOBAL_CMD.target_yaw_angle = IMU_data->Yaw;
			break;
		case AGV_State_Running_Auto_Down:
			AGV_GLOBAL_CMD.vx =-AGV_APPROACH_SPEED;
			AGV_GLOBAL_CMD.wz = 0;
			break;
		case AGV_State_Running_Auto_Up:

			PfsmSched_PostEvent(&ScanPlatform_Pfsm, PFSM_EVENT_FINISH_LOADPLATFORM);
			PfsmSched_Block(&StartMode_Pfsm); // 阻塞启动模式状态机
			break;
		default:
			return;
			break;
	}

}

void ScanPlatform_PfsmHandler(Pfsm_t *pfsm, PfsmEventId_e event) {
	Chassis_Speed_CalculateOfGray();
}

void Follow_Vision_PfsmHandler(Pfsm_t *pfsm, PfsmEventId_e event) {
	// 视觉跟随模式处理函数
	// static uint8_t last_state = 0xFF; // 0xFF 表示首次调用
	uint8_t state = (uint8_t) vision_recv_data->cmd_state;
	VisionBlockJudge(state);
	switch ((uint8_t) vision_recv_data->cmd_state) {
		case SEARCHING_TRAGET:
			PfsmSched_Block(&Follow_Vision_Pfsm); // 阻塞视觉跟随状态机,等待视觉锁定目标
			break;
		case TRACING:
			AGV_GLOBAL_CMD.target_yaw_angle = vision_recv_data->target_yaw;
			AGV_GLOBAL_CMD.vx = AGV_APPROACH_SPEED * 0.4;
			break;
		case SEARCH_TRABLE:
			AGV_GLOBAL_CMD.target_yaw_angle = vision_recv_data->target_yaw;
			AGV_GLOBAL_CMD.vx = vision_recv_data->car_speed;
			break;
		case READY_TO_PUSH:
			break;
		default:
			break;
	}
}

void AttackAvoid_PfsmHandler(Pfsm_t *pfsm, PfsmEventId_e event) {
	// 躲避袭击模式处理函数
}

void ReloadPlatform_PfsmHandler(Pfsm_t *pfsm, PfsmEventId_e event) {
	// 掉台后自动登台模式处理函数
}
