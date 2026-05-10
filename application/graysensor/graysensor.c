//
// Created by LIMBO on 2026/4/21.
//
#include "graysensor.h"
#include "bsp_log.h"
#include "cmsis_os.h"
#include "grayscale.h"
#include "message_center.h"
#include "usart.h"
// #include "vofa/vofa_usart.h"
// 消息发布订阅
static Publisher_t *Graysensor_Pub; // 发布灰度传感器的数据
static Subscriber_t *Graysensor_Sub; // 订阅灰度传感器的控制命令

static Graysensor_Ctrl_Cmd_s Graysensor_Cmd_Recv; // 接收到的控制命令
static Graysensor_Upload_Data_s Graysensor_Feedback_Data; // 反馈数据

static GrayscaleData_t *grayscale_data = NULL; // 灰度传感器数据指针


static GrayCalib_t GrayCalib[8]; // 灰度传感器的校准数据,最多支持6套校准数据


/*
 * @brief 灰度传感器校准数据初始化
 * 预设6套校准数据,分别对应X0与X2-X6一共六个通道的数据
 */
void GrayCalibInit() {
	//取测量值的5%-95%为归一化范围，忽略小于5%和大于95%的极端值对归一化的影响
	// X0通道数据
	GrayCalib[0].black_base = 3827; // 灰度0
	GrayCalib[0].white_base = 1976;
	//X1通道数据（暂时未使用）
	GrayCalib[1].black_base = 3862; // 灰度1
	GrayCalib[1].white_base = 2190;
	// X2通道数据
	GrayCalib[2].black_base = 3828; // 灰度2
	GrayCalib[2].white_base = 1916;
	// X3通道数据
	GrayCalib[3].black_base = 3835; // 灰度3
	GrayCalib[3].white_base = 1916;
	// X4通道数据
	GrayCalib[4].black_base = 3807; // 灰度4
	GrayCalib[4].white_base = 1782;

	// X5通道数据
	GrayCalib[5].black_base = 3815; // 灰度5
	GrayCalib[5].white_base = 1836;

	// X6通道数据
	GrayCalib[6].black_base = 3864; // 灰度6
	GrayCalib[6].white_base = 2197;

	//X7通道数据（暂时未使用）
	GrayCalib[7].black_base = 3855; // 灰度7
	GrayCalib[7].white_base = 2129;
}

/*
 * @brief 灰度传感器数据归一化
 * 根据预设的黑白基准值将原始传感器值归一化
 * */
void Gray_Normalize() {
	for (int i = 0; i < 8; i++) {
		Graysensor_Feedback_Data.sensor_Normalized[i] =
				(float) (Graysensor_Feedback_Data.sensor_values[i] - GrayCalib[i].white_base) / (float)
				(GrayCalib[i].black_base - GrayCalib[i].white_base);
	}
}

/*
 * @brief 灰度传感器应用初始化
 * 请在开启RTOS之前调用
 */
void GraysensorInit() {
	// 初始化灰度传感器 (使用USART8)
	extern UART_HandleTypeDef huart10;
	grayscale_data = GrayscaleInit(&huart10);

	// 初始化校准数据
	GrayCalibInit();
	// 注册消息发布订阅
	Graysensor_Sub = SubRegister("Graysensor_Cmd", sizeof(Graysensor_Ctrl_Cmd_s));
	Graysensor_Pub =
			PubRegister("Graysensor_Feed", sizeof(Graysensor_Upload_Data_s));

	// 发送默认控制命令 (模拟模式)
	GrayscaleSendControl(0, 1, 0);
}

/*
 * @brief 灰度传感器应用任务
 * 放入实时系统以一定频率运行
 */
void GraysensorTask() {
	// 获取控制命令
	SubGetMessage(Graysensor_Sub, &Graysensor_Cmd_Recv);

	if (tx_flag == 0) {
		GrayscaleSendControl(0, 1, 0);
	}

	// 根据控制命令设置传感器模式
	// GrayscaleSendControl(Graysensor_Cmd_Recv.calib_mode,
	//                     Graysensor_Cmd_Recv.analog_mode,
	//                     Graysensor_Cmd_Recv.digital_mode);

	// 检查是否有新数据包
	if (grayscale_data && grayscale_data->new_package_flag) {
		// 填充反馈数据
		for (int i = 0; i < 8; i++) {
			Graysensor_Feedback_Data.sensor_values[i] = grayscale_data->sensor_values[i];
		}
		Gray_Normalize();

		Graysensor_Feedback_Data.data_valid = grayscale_data->valid;
		Graysensor_Feedback_Data.sensor_online = GrayscaleIsOnline();

		// 清除标志位
		GrayscaleClearNewPackageFlag();

		// 使用阻塞发送灰度传感器数据
		// char tx_buf[128];
		// int len = sprintf(tx_buf, "%d,%d,%d,%d,%d,%d,%d,%d\r\n",
		//                   Graysensor_Feedback_Data.sensor_values[0],
		//                   Graysensor_Feedback_Data.sensor_values[1],
		//                   Graysensor_Feedback_Data.sensor_values[2],
		//                   Graysensor_Feedback_Data.sensor_values[3],
		//                   Graysensor_Feedback_Data.sensor_values[4],
		//                   Graysensor_Feedback_Data.sensor_values[5],
		//                   Graysensor_Feedback_Data.sensor_values[6],
		//                   Graysensor_Feedback_Data.sensor_values[7]);
		// HAL_UART_Transmit(&huart8, (uint8_t *)tx_buf, len, 100);
	}

	// osDelay(10);
	// 发布反馈数据
	PubPushMessage(Graysensor_Pub, (void *) &Graysensor_Feedback_Data);
}
