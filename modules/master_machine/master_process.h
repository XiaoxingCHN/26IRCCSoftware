#ifndef MASTER_PROCESS_H
#define MASTER_PROCESS_H

#include "bsp_usart.h"
#include "seasky_protocol.h"
#include "robot_cmd.h"
#define VISION_RECV_SIZE 18u // 当前为固定值,36字节
#define VISION_SEND_SIZE 36u

#pragma pack(1)// 对齐方式为1字节
typedef enum
{
	NO_FIRE = 0,
	AUTO_FIRE = 1,
	AUTO_AIM = 2
} Fire_Mode_e;

typedef enum
{
	NO_TARGET = 0,
	TARGET_CONVERGING = 1,
	READY_TO_FIRE = 2
} Target_State_e;

typedef enum
{
	NO_TARGET_NUM = 0,
	HERO1 = 1,
	ENGINEER2 = 2,
	INFANTRY3 = 3,
	INFANTRY4 = 4,
	INFANTRY5 = 5,
	OUTPOST = 6,
	SENTRY = 7,
	BASE = 8
} Target_Type_e;

typedef struct
{
	Fire_Mode_e fire_mode;
	Target_State_e target_state;
	Target_Type_e target_type;

	float target_yaw;
	float tracing_id;
        float cmd_state;
        float car_speed;
        // float NeedValue1;
        // float NeedValue2;
} Vision_Recv_s;

typedef enum {
	SEARCHING_TRAGET,
	TRACING,
	SEARCH_TRABLE,
	READY_TO_PUSH,
}Robot_Cmd_Vision_e;

typedef enum
{
	COLOR_NONE = 0,
	COLOR_BLUE = 1,
	COLOR_YELLOW = 2,
} Enemy_Color_e;

typedef enum
{
	VISION_MODE_AIM = 0,
	VISION_MODE_SMALL_BUFF = 1,
	VISION_MODE_BIG_BUFF = 2
} Work_Mode_e;

typedef enum
{
	BULLET_SPEED_NONE = 0,
	BIG_AMU_10 = 10,
	SMALL_AMU_15 = 15,
	BIG_AMU_16 = 16,
	SMALL_AMU_18 = 18,
	SMALL_AMU_30 = 30,
} Bullet_Speed_e;

typedef struct
{
	Enemy_Color_e enemy_color;
	Work_Mode_e work_mode;//模式选择
	Bullet_Speed_e bullet_speed;

	float yaw;
        float mcu_feed_state;//单片机返回在台上还是台下
        float laser_ranging_L0;
        float laser_ranging_L1;
        float laser_ranging_R0;
        float laser_ranging_R1;
        // float agv_mode;//底盘模式
} Vision_Send_s;
#pragma pack()

/**
 * @brief 调用此函数初始化和视觉的串口通信
 *
 * @param handle 用于和视觉通信的串口handle(C板上一般为USART1,丝印为USART2,4pin)
 */
Vision_Recv_s *VisionInit(UART_HandleTypeDef *_handle);

/**
 * @brief 发送视觉数据
 *
 */
void VisionSend();

/**
 * @brief 设置视觉发送标志位
 *
 * @param enemy_color
 * @param work_mode
 * @param bullet_speed
 */
void VisionSetFlag(Enemy_Color_e enemy_color, Work_Mode_e work_mode, Bullet_Speed_e bullet_speed);

/**
 * @brief 设置发送数据的姿态部分
 *
 * @param yaw
 * @param pitch
 */
void VisionSetAltitude(float yaw);

/**
 * @brief 设置发送数据的激光距离部分
 *
 * @param laser_ranging_L0 左激光距离
 * @param laser_ranging_L1 左激光距离1
 * @param laser_ranging_R0 右激光距离0
 * @param laser_ranging_R1 右激光距离1
 */
void VisionSetLaserRanging(float laser_ranging_L0, float laser_ranging_L1, float laser_ranging_R0, float laser_ranging_R1);

/**
 *  @brief 设置发送数据的小车状态
 * @param Robot_Position 单片机当前状态,在台上还是台下,用于视觉决策
 */
void VisionSetState(float Robot_Position) ;
#endif // !MASTER_PROCESS_H