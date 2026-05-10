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
#define YAW_ALIGN_ANGLE \
  (YAW_CHASSIS_ALIGN_ECD * ECD_ANGLE_COEF_DJI)  // 对齐时的角度,0-360
#define PTICH_HORIZON_ANGLE \
  (PITCH_HORIZON_ECD * ECD_ANGLE_COEF_DJI)  // pitch水平时电机的角度,0-360
/*消息中心*/
static Publisher_t *Chassis_Cmd_Pub; // 底盘控制消息发布者
static Subscriber_t *Chassis_Feed_Sub; // 底盘反馈信息订阅者

static Chassis_Ctrl_Cmd_s
Chassis_Cmd_Send; // 发送给底盘应用的信息，包括控制信息
static Chassis_Upload_Data_s
Chassis_Fetch_Data; // 从底盘应用接收的反馈信息与底盘的运动状态

// 灰度传感器消息中心
static Publisher_t *Graysensor_Cmd_Pub; // 灰度传感器控制消息发布者
static Subscriber_t *Graysensor_Feed_Sub; // 灰度传感器反馈信息订阅者

static Graysensor_Ctrl_Cmd_s Graysensor_Cmd_Send; // 发送给灰度传感器的控制信息
static Graysensor_Upload_Data_s
Graysensor_Fetch_Data; // 从灰度传感器接收的反馈信息

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

static AGV_Control_Cmd_s AGV_GLOBAL_CMD; // AGV模式下才会启用的全局模式
// AGV模式下的相关参数
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
	rc_data = RemoteControlInit(
		&huart5); // 修改为对应串口,注意如果是自研板dbus协议串口需选用添加了反相器的那个
	vision_recv_data = VisionInit(&huart9); // 视觉通信串口

	Target_Yaw_Angele = IMU_data->Yaw;

	Chassis_Cmd_Pub = PubRegister("Chassis_Cmd", sizeof(Chassis_Ctrl_Cmd_s));
	Chassis_Feed_Sub = SubRegister("Chassis_Feed", sizeof(Chassis_Upload_Data_s));

	// 注册灰度传感器消息
	Graysensor_Cmd_Pub =
			PubRegister("Graysensor_Cmd", sizeof(Graysensor_Ctrl_Cmd_s));
	Graysensor_Feed_Sub =
			SubRegister("Graysensor_Feed", sizeof(Graysensor_Upload_Data_s));

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
		Robot_State_Mode_Global = CONTROL_AGV;
	}
}

static void VisionSend_Data(void) {
	VisionSetAltitude(IMU_data->Yaw);
	// 将TOF200C的测距数据发送给上位机作为补充
	VisionSetLaserRanging(
		TOF050C_Fetch_Data.range_values[4], TOF050C_Fetch_Data.range_values[5],
		TOF050C_Fetch_Data.range_values[6], TOF050C_Fetch_Data.range_values[7]);
}

static void Chassis_Control_Send(void) {
	switch (Chassis_Cmd_Send.chassis_mode) {
		case CHASSIS_ZERO_FORCE:
			Chassis_Cmd_Send.vx = 0;
			Chassis_Cmd_Send.wz = 0;
			break;
		case CHASSIS_NORMAL:
			switch (Robot_State_Mode_Global) {
				case CONTROL_REMOTE:
					Chassis_Cmd_Send.vx = rc_data[TEMP].rc.rocker_l1 *0.6f;
					Chassis_Cmd_Send.wz = rc_data[TEMP].rc.rocker_r_ * 0.4f;
					break;
				case CONTROL_AGV:
					Chassis_Cmd_Send.vx = AGV_GLOBAL_CMD.vx;
					Chassis_Cmd_Send.wz = AGV_GLOBAL_CMD.wz;
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

// /**
//  * @brief 上台状态机
//  */
// static void UPdonePlatform(void) {
//     typedef enum {
//         STAGE_START,//启动模式
//         STAGE_APPROACH,   // 接近平台
//         STAGE_CLIMB,      // 中速冲台（vx=AGV_START_SPEED）
//         STAGE_SLOW_CLIMB, // 低速爬台（vx=UPLOAD_PLATFORM_SPEED）
//         STAGE_RETRY,      // 失败 → 后退 + 回转重试
//         STAGE_DONE,       // 上台完成
//     } ClimbStage_e;
//
//     static ClimbStage_e climb_stage = STAGE_APPROACH;
//     static float start_yaw = 0;
//     static uint32_t stage_start_ms = 0;
//     static uint32_t gray_ok_ms = 0;
//     static uint8_t retry_cnt = 0;
//
//     // 读取来自上位机的距离扫描值（后档方向）
//   float BackToPlatformDistance = 9999.0f; //
//   修复：必须给一个超大默认值，绝不能是0，否则上位机离线时会自动瞬间通过所有的测距判定触发疯冲！
//   // if (vision_recv_data->NeedValue1!=0) {
//   //   BackToPlatformDistance=vision_recv_data->NeedValue1;
//   // }
//
//
//     // 灰度全部低于阈值
//     bool all_gray_below = true;
//     for (int i = 0; i < 8; i++) {
//         if (Graysensor_Fetch_Data.sensor_Normalized[i] > 0.3f) {
//             all_gray_below = false;
//             break;
//         }
//     }
//
//     float pitch = IMU_data->Pitch;
//     float yaw_change = fabsf(IMU_data->Yaw - start_yaw);
//     if (yaw_change > 180.0f) yaw_change = 360.0f - yaw_change;
//
//     uint32_t now_ms = DWT_GetTimeline_ms();
//
//     switch (climb_stage) {
//
//       case STAGE_START://测试阶段用遥控拨轮代替双边无接触启动
//       if (rc_data[TEMP].rc.dial >= 300) {
//             climb_stage = STAGE_APPROACH;
//             start_yaw = IMU_data->Yaw;
//             stage_start_ms = now_ms;
//         }
//       break;
//     case STAGE_APPROACH:
//       //接近平台
//         if (BackToPlatformDistance <= UPLOAD_PLATFORM_DISTANCE) {
//             climb_stage = STAGE_CLIMB;
//             start_yaw = IMU_data->Yaw;
//             stage_start_ms = now_ms;
//         } else {
//             // 没到台前，慢速前进找台
//             Chassis_Cmd_Send.vx = AGV_APPROACH_SPEED;
//             Chassis_Cmd_Send.target_yaw_angle = IMU_data->Yaw;
//         }
//         break;
//
//     case STAGE_CLIMB:
//         // 超时保护
//         if (now_ms - stage_start_ms > CLIMB_TIMEOUT_MS) {
//             climb_stage = STAGE_RETRY;
//             stage_start_ms = now_ms;
//             break;
//         }
//         // 继续冲台
//         Chassis_Cmd_Send.vx = AGV_START_SPEED;
//         Chassis_Cmd_Send.target_yaw_angle = vision_recv_data->target_yaw;
//         if (BackToPlatformDistance<=UPLOAD_PLATFORM_LOWER_THRESHOLD) {
//           climb_stage = STAGE_SLOW_CLIMB;
//           stage_start_ms = now_ms;
//         }
//         break;
//
//     case STAGE_SLOW_CLIMB:
//         // 超时保护
//         if (now_ms - stage_start_ms > CLIMB_TIMEOUT_MS) {
//             climb_stage = STAGE_RETRY;
//             stage_start_ms = now_ms;
//             break;
//         }
//
//         // 灰度连续达标 → 上台成功
//         if (all_gray_below) {
//             if (gray_ok_ms == 0) {
//                 gray_ok_ms = now_ms;
//             } else if (now_ms - gray_ok_ms >= GRAY_CONFIRM_MS) {
//                 climb_stage = STAGE_DONE;
//                 break;
//             }
//         } else {
//             gray_ok_ms = 0; // 灰度未达标，复位计时
//         }
//         // 继续爬台
//         Chassis_Cmd_Send.vx = UPLOAD_PLATFORM_SPEED;
//         Chassis_Cmd_Send.target_yaw_angle = IMU_data->Yaw;
//         break;
//
//     case STAGE_RETRY:
//         retry_cnt++;
//         if (retry_cnt >= MAX_RETRY_COUNT) {
//             // 重试耗尽，强制认为上台完成（保底）
//             climb_stage = STAGE_DONE;
//             break;
//         }
//         // 后退到安全距离
//         if (BackToPlatformDistance > UPLOAD_PLATFORM_DISTANCE + 5.0f) {
//             // 退够了，重新接近
//             climb_stage = STAGE_APPROACH;
//         } else {
//             Chassis_Cmd_Send.vx = -UPLOAD_PLATFORM_SPEED;
//             Chassis_Cmd_Send.wz = 0.3f;
//             Chassis_Cmd_Send.target_yaw_angle = start_yaw;
//         }
//         break;
//
//     case STAGE_DONE:
//         AGV_MODE_GLOBAL.StartFlag = true;
//         AGV_MODE_GLOBAL.Mode = AGV_Mode_SCANPLATFORM;
//         AGV_MODE_GLOBAL.Priority = 2;
//         // 复位状态机，下次不再进入
//         climb_stage = STAGE_APPROACH;
//         retry_cnt = 0;
//         break;
//     }
// }
// /*
//  *  @brief 检测是否掉台
//  *  @return true 掉台  false 没掉台
//  */
// static bool UnderPlatformDetect(void) {
//
//   bool UnderPlatformFlag = false;
//   for (int i = 0; i < 8; i++) {
//     if (Graysensor_Fetch_Data.sensor_Normalized[i] > 0.8f) {
//       UnderPlatformFlag = true;
//       break;
//     }
//   }
//   return UnderPlatformFlag;
// }
//
// /**
//  * @brief ModeJudge — 优先级调度器
//  *        根据传感器条件选出最高优先级就绪模式
//  */
// static void ModeJudge(void) {
//   if (Chassis_Cmd_Send.chassis_mode != CHASSIS_AGV_MODE) {
//     // 非AGV模式，不干预
//     return;
//   }
//
//   /* 首次启动 → StartMode */
//   if (AGV_MODE_GLOBAL.StartFlag == false) {
//     AGV_MODE_GLOBAL.Mode = AGV_Mode_Start;
//     AGV_MODE_GLOBAL.Priority = 10;
//     return;
//   }
//
//   /* ===== 正常决策：从高到低检查各模式条件 ===== */
//   AGV_Mode_e best_mode = AGV_Mode_ZERO_FORCE;
//   uint8_t best_priority = 0;
//
//   static uint32_t fall_recover_timer = 0;
//
//   //P10：掉台检测后台运行一旦检测到立即介入启动返回平台，且维持硬退500ms避免在边缘抽搐震荡打滑
//   if (UnderPlatformDetect()) {
//     best_mode = AGV_Mode_RETURNPLATFORM;
//     best_priority = 10;
//     goto done;
//   }
//
//
//   // P9：敌方袭击（视觉检测到敌方）
//   // if (/* 敌方袭击条件 */) {
//   //     best_mode = AGV_Mode_ATTACK;
//   //     best_priority = 9;
//   //     goto done;
//   // }
//
//   // P8：检测到能量块
//   if (vision_recv_data->tracing_id!=-1) {
//       best_mode = AGV_Mode_FLLOW;
//       best_priority = 8;
//       goto done;
//   }
//
//   // P7：检测到对方机器人
//   // if (vision_recv_data->target_yaw != 0) {
//   //     best_mode = AGV_Mode_ATTACK;
//   //     best_priority = 7;
//   //     goto done;
//   // }
//
//   // 默认：自动巡台
//   best_mode = AGV_Mode_SCANPLATFORM;
//   best_priority = 2;
//
// done:
//   AGV_MODE_GLOBAL.Mode = best_mode;
//   AGV_MODE_GLOBAL.Priority = best_priority;
//
// }
//
// /**
//  * @brief AutoPatrolTask — 自动巡台任务（流程图下方）
//  *        灰度归一化 + 加权算速度 + TOF边缘检测 + 转向角选择
//  */
// static void AutoPatrolTask(void) {
// //检测四角对地距离，判断是否检测到边缘（需调参）
//   bool edge_detected = false;
//     if (TOF050C_Fetch_Data.range_values[0]>AGV_LASER_DISTANCE_FL ||
//     TOF050C_Fetch_Data.range_values[1]>AGV_LASER_DISTANCE_FR ||
//         TOF050C_Fetch_Data.range_values[2]>AGV_LASER_DISTANCE_BL ||
//         TOF050C_Fetch_Data.range_values[3]>AGV_LASER_DISTANCE_BR) {
//         edge_detected = true;
//     }
// /*Begin 灰度计算速度*/
//     float gray_norm[8];
//     for (int i = 0; i < 8; i++) {
//         gray_norm[i] = Graysensor_Fetch_Data.sensor_Normalized[i];
//     }
//     float avg_gray = 0;
//     for (int i = 0; i < 8; i++) avg_gray += gray_norm[i];
//     avg_gray /= 8.0f;
//     // 灰度0→最快, 灰度1→最慢(停)
//     float speed = AGV_APPROACH_SPEED * (1.0f - avg_gray);
//     Chassis_Cmd_Send.vx = (speed < 0.05f) ? 0.05f : speed;
// /*End 灰度计算速度*/
//
//     static bool is_turning = false;
//     static float turn_target_yaw = 0;
//
//     // 修复：检测转向动作是否完成
//     if (is_turning) {
//         float err = fabsf(turn_target_yaw - IMU_data->Yaw);
//         if (err > 180.0f) err = 360.0f - err;
//         if (err < 10.0f) {
//             is_turning = false; // 误差进入10度以内，转向宣告完成，去锁
//         }
//     }
//
//     /*Begin 灰度+Yaw 判断当前位置*/
//     bool on_diagonal_edge = false;
//     // if (/* 灰度+Yaw判定在对角边 */) { on_diagonal_edge = true; }
//
//     if (edge_detected || avg_gray > 0.7f) {
//         bool yaw_point_outward = false;
//         // if (/* 判断当前Yaw是否向外 */) { yaw_point_outward = true; }
//
//         if (!yaw_point_outward && !is_turning) {
//             // 修复：锁存状态！只计算��次目标，防止 200hz 频率循环无限累加
//             Yaw 引起无故疯转陀螺 is_turning = true; if (on_diagonal_edge) {
//                 // 对角边 → 转120°
//                 turn_target_yaw = IMU_data->Yaw + 120.0f;
//             } else {
//                 // 直边 → 转90°
//                 turn_target_yaw = IMU_data->Yaw + 90.0f;
//             }
//         }
//         // 如果Yaw已经向外，不转方向继续走
//     }
//
//     // 6. 检查方向：灰度值是否在减小（说明在往白色区域走→方向对）
//     //    需保存上一帧灰度值进行比较：若灰度增大(变黑→方向错)，回退重算
//     // static float last_avg_gray = 0;
//     // if (avg_gray > last_avg_gray) {
//     //     // 方向不对，调头或重算
//     // }
//     // last_avg_gray = avg_gray;
//
//     // 修复：只有平时保持直行才覆盖为 Yaw，如果有正在转向的动作则维持执行
//     if (is_turning) {
//         Chassis_Cmd_Send.target_yaw_angle = turn_target_yaw;
//     } else {
//         Chassis_Cmd_Send.target_yaw_angle = IMU_data->Yaw;
//     }
//     YawAngleOFFSET();
// }
//
// /**
//  * @brief AGV_Mode_Switch — 根据当前模式执行对应的行为函数
//  */
// static void AGV_Mode_Switch(void) {
//   switch (AGV_MODE_GLOBAL.Mode) {
//   case AGV_Mode_Start:
//     UPdonePlatform();
//     break;
//   case AGV_Mode_SCANPLATFORM:
//     AutoPatrolTask();
//     break;
//   case AGV_Mode_FLLOW:
//     // 视觉跟随模式
//     if (vision_recv_data->target_yaw != 0) {
//         Chassis_Cmd_Send.target_yaw_angle = vision_recv_data->target_yaw;
//     }
//     Chassis_Cmd_Send.vx = 0.5f;
//     Chassis_Cmd_Send.chassis_mode = CHASSIS_AGV_MODE;
//     break;
//   case AGV_Mode_ATTACK:
//     // 攻击模式
//     if (vision_recv_data->target_yaw != 0) {
//         Chassis_Cmd_Send.vx = 1.5f;
//         Chassis_Cmd_Send.target_yaw_angle = vision_recv_data->target_yaw;
//         Chassis_Cmd_Send.chassis_mode = CHASSIS_AGV_MODE;
//     }
//     break;
//   case AGV_Mode_OBSTACLE:
//     // 避障模式
//     Chassis_Cmd_Send.vx = -0.5f;
//     Chassis_Cmd_Send.wz = 0.8f;
//     Chassis_Cmd_Send.chassis_mode = CHASSIS_NORMAL;
//     Chassis_Cmd_Send.target_yaw_angle = IMU_data->Yaw;
//     break;
//   case AGV_Mode_ANTIDROP:
//     // 防掉落模式
//     Chassis_Cmd_Send.vx = 0;
//     Chassis_Cmd_Send.wz = 0;
//     // Chassis_Cmd_Send.chassis_mode = CHASSIS_ZERO_FORCE;
//     break;
//   case AGV_Mode_RETURNPLATFORM:
//     // 掉台返回模式
//     Chassis_Cmd_Send.vx = -0.8f;
//     Chassis_Cmd_Send.chassis_mode = CHASSIS_NORMAL;
//     Chassis_Cmd_Send.target_yaw_angle = IMU_data->Yaw;
//     break;
//   case AGV_Mode_ZERO_FORCE:
//   default:
//     // Chassis_Cmd_Send.chassis_mode = CHASSIS_ZERO_FORCE;
//     break;
//   }
// }

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
	PfsmSched_Run(); // 运行状态机调度器，自动根据优先级执行对应的状态机处理函数
	// /* ===== AGV 决策链：先选模式 → 再执行模式 ===== */
	// if (Chassis_Cmd_Send.chassis_mode == CHASSIS_AGV_MODE) {
	//   ModeJudge();        // Step 1: 根据传感器条件选模式
	//   AGV_Mode_Switch();  // Step 2: 根据模式填控制命令
	// }

	/*Control Code End*/
	// 发布底盘命令与灰度传感器命令
	PubPushMessage(Chassis_Cmd_Pub, (void *) &Chassis_Cmd_Send);
	PubPushMessage(Graysensor_Cmd_Pub, (void *) &Graysensor_Cmd_Send);
	PubPushMessage(TOF050C_Cmd_Pub, (void *) &TOF050C_Cmd_Send);
}

void StartMode_PfsmHandler(Pfsm_t *pfsm, PfsmEventId_e event) {
	PfsmSched_PostEvent(&ScanPlatform_Pfsm, PFSM_EVENT_FINISH_LOADPLATFORM);
}

void ScanPlatform_PfsmHandler(Pfsm_t *pfsm, PfsmEventId_e event) {
	switch (event) {
		case PFSM_EVENT_FINISH_LOADPLATFORM:
			// 上台完成，进入巡台模式

			break;
		default:
			break;
	}
}

void Follow_Vision_PfsmHandler(Pfsm_t *pfsm, PfsmEventId_e event) {
	// 视觉跟随模式处理函数
}

void AttackAvoid_PfsmHandler(Pfsm_t *pfsm, PfsmEventId_e event) {
	// 躲避袭击模式处理函数
}

void ReloadPlatform_PfsmHandler(Pfsm_t *pfsm, PfsmEventId_e event) {
	// 掉台后自动登台模式处理函数
}


