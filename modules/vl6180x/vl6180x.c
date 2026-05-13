//
// Created by LIMBO on 2026/4/23.
//
#include "vl6180x.h"

#include "bsp_dwt.h"
#include "bsp_iic.h"
#include "cmsis_os.h"
#include "main.h"
#include "stdio.h"

extern IICInstance *tof_iic; // BSP I2C 实例（由 TOF_Sensors.c 初始化）

// 修改为正确的默认地址�?x29对应�?位地址�?
#define VL6180X_DEFAULT_ADDRESS   0x29  // 7位地址

// 动态地址变量，默认初始化�?x29
uint8_t vl6180x_current_address = VL6180X_DEFAULT_ADDRESS;

// 动态计算写/读地址
//#define addr_write   (vl6180x_current_address << 1)
//#define addr_read    (vl6180x_current_address << 1 | 0x01)


#define IDENTIFICATION__MODEL_ID                0x000
#define IDENTIFICATION__MODEL_REV_MAJOR       0x001
#define IDENTIFICATION__MODEL_REV_MINOR       0x002
#define IDENTIFICATION__MODULE_REV_MAJOR      0x003
#define IDENTIFICATION__MODULE_REV_MINOR      0x004
#define IDENTIFICATION__DATE_HI               0x006
#define IDENTIFICATION__DATE_LO               0x007
#define IDENTIFICATION__TIME                  0x008
#define SYSTEM__MODE_GPIO0                    0x010
#define SYSTEM__MODE_GPIO1                    0x011
#define SYSTEM__HISTORY_CTRL                  0x012
#define SYSTEM__INTERRUPT_CONFIG_GPIO         0x014
#define SYSTEM__INTERRUPT_CLEAR               0x015
#define SYSTEM__FRESH_OUT_OF_RESET            0x016
#define SYSTEM__GROUPED_PARAMETER_HOLD        0x017
#define SYSRANGE__START                       0x018
#define SYSRANGE__THRESH_HIGH                 0x019
#define SYSRANGE__THRESH_LOW                  0x01A
#define SYSRANGE__INTERMEASUREMENT_PERIOD     0x01B
#define SYSRANGE__MAX_CONVERGENCE_TIME        0x01C
#define SYSRANGE__CROSSTALK_COMPENSATION_RATE 0x01E
#define SYSRANGE__CROSSTALK_VALID_HEIGHT      0x021
#define SYSRANGE__EARLY_CONVERGENCE_ESTIMATE  0x022
#define SYSRANGE__PART_TO_PART_RANGE_OFFSET   0x024
#define SYSRANGE__RANGE_IGNORE_VALID_HEIGHT   0x025
#define SYSRANGE__RANGE_IGNORE_THRESHOLD      0x026
#define SYSRANGE__MAX_AMBIENT_LEVEL_MULT      0x02C
#define SYSRANGE__RANGE_CHECK_ENABLES         0x02D
#define SYSRANGE__VHV_RECALIBRATE             0x02E
#define SYSRANGE__VHV_REPEAT_RATE             0x031
#define SYSALS__START                         0x038
#define SYSALS__THRESH_HIGH                   0x03A
#define SYSALS__THRESH_LOW                    0x03C
#define SYSALS__INTERMEASUREMENT_PERIOD       0x03E
#define SYSALS__ANALOGUE_GAIN                 0x03F
#define SYSALS__INTEGRATION_PERIOD            0x040
#define RESULT__RANGE_STATUS                  0x04D
#define RESULT__ALS_STATUS                    0x04E
#define RESULT__INTERRUPT_STATUS_GPIO         0x04F
#define RESULT__ALS_VAL                       0x050
#define RESULT__HISTORY_BUFFER_0              0x052
#define RESULT__HISTORY_BUFFER_1              0x054
#define RESULT__HISTORY_BUFFER_2              0x056
#define RESULT__HISTORY_BUFFER_3              0x058
#define RESULT__HISTORY_BUFFER_4              0x05A
#define RESULT__HISTORY_BUFFER_5              0x05C
#define RESULT__HISTORY_BUFFER_6              0x05E
#define RESULT__HISTORY_BUFFER_7              0x060
#define RESULT__RANGE_VAL                     0x062
#define RESULT__RANGE_RAW                     0x064
#define RESULT__RANGE_RETURN_RATE             0x066
#define RESULT__RANGE_REFERENCE_RATE          0x068
#define RESULT__RANGE_RETURN_SIGNAL_COUNT     0x06C
#define RESULT__RANGE_REFERENCE_SIGNAL_COUNT  0x070
#define RESULT__RANGE_RETURN_AMB_COUNT        0x074
#define RESULT__RANGE_REFERENCE_AMB_COUNT     0x078
#define RESULT__RANGE_RETURN_CONV_TIME        0x07C
#define RESULT__RANGE_REFERENCE_CONV_TIME     0x080
#define RANGE_SCALER                          0x096
#define READOUT__AVERAGING_SAMPLE_PERIOD      0x10A
#define FIRMWARE__BOOTUP                      0x119
#define FIRMWARE__RESULT_SCALER               0x120
#define VL6180_I2C_SLAVE_DEVICE_ADDRESS 	  0x0212
#define INTERLEAVED_MODE__ENABLE              0x2A3
#define VL6180_DEFAULT_ADDRESS          	  0x29

const uint16_t ScalerValues[] = {0, 253, 127, 84};
uint8_t ptp_offset;
uint8_t scaling;
uint16_t io_timeout1;

/* ---- BSP I2C 封装：VL6180X 使用 16-bit 寄存器地址 ---- */
static inline void tof6180_mem_write(uint8_t addr_write, uint16_t reg,
                                     uint8_t *data, uint16_t size) {
	IICSetDeviceAddress(tof_iic, addr_write >> 1); // HAL格式→7-bit
	IICAccessMem(tof_iic, reg, data, size, IIC_WRITE_MEM, 0); // 16-bit mem
}

static inline uint8_t tof6180_mem_read(uint8_t addr_write, uint16_t reg) {
	uint8_t data = 0;
	IICSetDeviceAddress(tof_iic, addr_write >> 1);
	IICAccessMem(tof_iic, reg, &data, 1, IIC_READ_MEM, 0);
	return data;
}

void VL6180X_WR_CMD(uint16_t cmd, uint8_t data, uint8_t addr_write) {
	tof6180_mem_write(addr_write, cmd, &data, 1);
}

void VL6180X_WR_CMD2(uint16_t cmd, uint16_t data, uint8_t addr_write) {
	uint8_t buf[2] = {(data >> 8) & 0xff, data & 0xff};
	tof6180_mem_write(addr_write, cmd, buf, 2);
}

uint8_t VL6180X_ReadByte(uint16_t reg, uint8_t addr_write, uint8_t addr_read) {
	(void) addr_read; // BSP IICAccessMem 自动处理 R/W 位
	return tof6180_mem_read(addr_write, reg);
}

uint8_t VL6180X_Init(uint8_t addr_write, uint8_t addr_read) {
	ptp_offset = 0;
	scaling = 0;
	io_timeout1 = 2; // 重试次数（非毫秒！BSP I2C 已提供 100ms 硬件超时）

	ptp_offset = VL6180X_ReadByte(SYSRANGE__PART_TO_PART_RANGE_OFFSET, addr_write, addr_read);
	uint8_t reset = VL6180X_ReadByte(0x016, addr_write, addr_read); //check wether reset over
	if (reset == 1) {
		VL6180X_WR_CMD(0X0207, 0X01, addr_write);
		VL6180X_WR_CMD(0X0208, 0X01, addr_write);
		VL6180X_WR_CMD(0X0096, 0X00, addr_write);
		VL6180X_WR_CMD(0X0097, 0XFD, addr_write);
		VL6180X_WR_CMD(0X00E3, 0X00, addr_write);
		VL6180X_WR_CMD(0X00E4, 0X04, addr_write);
		VL6180X_WR_CMD(0X00E5, 0X02, addr_write);
		VL6180X_WR_CMD(0X00E6, 0X01, addr_write);
		VL6180X_WR_CMD(0X00E7, 0X03, addr_write);
		VL6180X_WR_CMD(0X00F5, 0X02, addr_write);
		VL6180X_WR_CMD(0X00D9, 0X05, addr_write);
		VL6180X_WR_CMD(0X00DB, 0XCE, addr_write);
		VL6180X_WR_CMD(0X02DC, 0X03, addr_write);
		VL6180X_WR_CMD(0X00DD, 0XF8, addr_write);
		VL6180X_WR_CMD(0X009F, 0X00, addr_write);
		VL6180X_WR_CMD(0X00A3, 0X3C, addr_write);
		VL6180X_WR_CMD(0X00B7, 0X00, addr_write);
		VL6180X_WR_CMD(0X00BB, 0X3C, addr_write);
		VL6180X_WR_CMD(0X00B2, 0X09, addr_write);
		VL6180X_WR_CMD(0X00CA, 0X09, addr_write);
		VL6180X_WR_CMD(0X0198, 0X01, addr_write);
		VL6180X_WR_CMD(0X01B0, 0X17, addr_write);
		VL6180X_WR_CMD(0X01AD, 0X00, addr_write);
		VL6180X_WR_CMD(0X00FF, 0X05, addr_write);
		VL6180X_WR_CMD(0X0100, 0X05, addr_write);
		VL6180X_WR_CMD(0X0199, 0X05, addr_write);
		VL6180X_WR_CMD(0X01A6, 0X1B, addr_write);
		VL6180X_WR_CMD(0X01AC, 0X3E, addr_write);
		VL6180X_WR_CMD(0X01A7, 0X1F, addr_write);
		VL6180X_WR_CMD(0X0030, 0X00, addr_write);

		VL6180X_WR_CMD(0X0011, 0X10, addr_write);
		VL6180X_WR_CMD(0X010A, 0X30, addr_write);
		VL6180X_WR_CMD(0X003F, 0X46, addr_write);
		VL6180X_WR_CMD(0X0031, 0XFF, addr_write);
		VL6180X_WR_CMD(0X0040, 0X63, addr_write);
		VL6180X_WR_CMD(0X002E, 0X01, addr_write);
		VL6180X_WR_CMD(0X001B, 0X09, addr_write);
		VL6180X_WR_CMD(0X003E, 0X31, addr_write);
		VL6180X_WR_CMD(0X0014, 0X24, addr_write);

		//        VL6180X_WR_CMD(0x016,0x00,addr_write);
		return 1;
	}
	return 0;
}


void VL6180X_SetScaling(uint8_t new_scaling, uint8_t addr_write, uint8_t addr_read) {
	uint8_t const DefaultCrosstalkValidHeight = 20; // default value of SYSRANGE__CROSSTALK_VALID_HEIGHT

	// do nothing if scaling value is invalid
	if (new_scaling < 1 || new_scaling > 3) { return; }

	scaling = new_scaling;
	VL6180X_WR_CMD2(RANGE_SCALER, ScalerValues[scaling], addr_write);

	// apply scaling on part-to-part offset
	VL6180X_WR_CMD(SYSRANGE__PART_TO_PART_RANGE_OFFSET, ptp_offset / scaling, addr_write);

	// apply scaling on CrossTalkValidHeight
	VL6180X_WR_CMD(SYSRANGE__CROSSTALK_VALID_HEIGHT, DefaultCrosstalkValidHeight / scaling, addr_write);

	// This function does not apply scaling to RANGE_IGNORE_VALID_HEIGHT.

	// enable early convergence estimate only at 1x scaling
	uint8_t rce = VL6180X_ReadByte(SYSRANGE__RANGE_CHECK_ENABLES, addr_write, addr_read);
	VL6180X_WR_CMD(SYSRANGE__RANGE_CHECK_ENABLES, (rce & 0xFE) | (scaling == 1), addr_write);
}


void VL6180X_ConfigureDefault(uint8_t addr_write, uint8_t addr_read) {
	VL6180X_WR_CMD(READOUT__AVERAGING_SAMPLE_PERIOD, 0x30, addr_write);
	VL6180X_WR_CMD(SYSALS__ANALOGUE_GAIN, 0x46, addr_write);
	VL6180X_WR_CMD(SYSRANGE__VHV_REPEAT_RATE, 0xFF, addr_write);
	VL6180X_WR_CMD2(SYSALS__INTEGRATION_PERIOD, 0x0063, addr_write);
	VL6180X_WR_CMD(SYSRANGE__VHV_RECALIBRATE, 0x01, addr_write);
	VL6180X_WR_CMD(SYSRANGE__INTERMEASUREMENT_PERIOD, 0x09, addr_write);
	VL6180X_WR_CMD(SYSALS__INTERMEASUREMENT_PERIOD, 0x31, addr_write);
	VL6180X_WR_CMD(SYSTEM__INTERRUPT_CONFIG_GPIO, 0x24, addr_write);
	VL6180X_WR_CMD(SYSRANGE__MAX_CONVERGENCE_TIME, 0x31, addr_write);
	VL6180X_WR_CMD(INTERLEAVED_MODE__ENABLE, 0, addr_write);
	VL6180X_SetScaling(1, addr_write, addr_read);
}

void VL6180X_SetTimeout(uint16_t timeout) {
	io_timeout1 = timeout;
}

uint8_t VL6180X_Start_Range(uint8_t addr_write, uint8_t addr_read) {
	VL6180X_WR_CMD(0x018, 0x01, addr_write);
	return 0;
}

uint16_t timeoutcnt = 0;

/*poll for new sample ready */
uint8_t VL6180X_Poll_Range(uint8_t addr_write, uint8_t addr_read) {
	uint8_t status;
	uint8_t range_status;
	timeoutcnt = 0; // 每个 sensor 独立计数
	status = VL6180X_ReadByte(0x04f, addr_write, addr_read);
	range_status = status & 0x07;
	while (range_status != 0x04) {
		timeoutcnt++;
		if (timeoutcnt > io_timeout1) {
			break;
		}
		status = VL6180X_ReadByte(0x04f, addr_write, addr_read);
		range_status = status & 0x07;
		osDelay(1);
	}
	return 0;
}


/*read range result (mm)*/
uint8_t VL6180_Read_Range(uint8_t addr_write, uint8_t addr_read) {
	int range;
	range = VL6180X_ReadByte(0x062, addr_write, addr_read);
	return range;
}

/*clear interrupt*/
void VL6180X_Clear_Interrupt(uint8_t addr_write, uint8_t addr_read) {
	VL6180X_WR_CMD(0x015, 0x07, addr_write);
}

void VL6180X_ReadRangeMultiple(const TOF_050C_WRREG regs[],
                               uint16_t results[], uint8_t count) {
	for (uint8_t i = 0; i < count; i++)
		VL6180X_Start_Range(regs[i].addr_w, regs[i].addr_r);

	for (uint8_t i = 0; i < count; i++)
		VL6180X_Poll_Range(regs[i].addr_w, regs[i].addr_r);
	osDelay(20);
	for (uint8_t i = 0; i < count; i++)
		results[i] = (uint16_t) scaling * VL6180_Read_Range(regs[i].addr_w, regs[i].addr_r);
}

uint16_t VL6180X_ReadRangeSingleMillimeters(uint8_t addr_write, uint8_t addr_read) {
	/*Start Single measure mode*/
	VL6180X_Start_Range(addr_write, addr_read);
	/* Wait for measurement ready. */
	VL6180X_Poll_Range(addr_write, addr_read);
	osDelay(20);
	return (uint16_t) scaling * VL6180_Read_Range(addr_write, addr_read);
}

/**
  * @brief  读取VL6180寄存�?
  * @param  dev_addr: 传感器当前地址
  * @param  reg_addr: 目标寄存器地址
  * @param  data:     读取的数据指�?
  * @retval HAL_OK: 成功 | HAL_ERROR: 失败
  */
HAL_StatusTypeDef VL6180_ReadRegister(uint8_t dev_addr, uint16_t reg_addr, uint8_t *data) {
	IICSetDeviceAddress(tof_iic, dev_addr);
	IICAccessMem(tof_iic, reg_addr, data, 1, IIC_READ_MEM, 0);
	return HAL_OK;
}

void VL6180x_ChangeAddress(uint8_t new_address_7bit) {
	uint8_t data = new_address_7bit;
	IICSetDeviceAddress(tof_iic, 0x29);
	IICAccessMem(tof_iic, 0x0212, &data, 1, IIC_WRITE_MEM, 0);
}

void AddressTest() {
	for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
		IICSetDeviceAddress(tof_iic, addr);
		HAL_StatusTypeDef status = HAL_I2C_IsDeviceReady(tof_iic->handle, addr << 1, 1, 10);
		if (status == HAL_OK) {
			printf("Device found at 0x%02X\n", addr);
		}
	}
}

void multisensor_vl6180x() {
	HAL_GPIO_WritePin(ID1_GPIO_Port, ID1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(ID2_GPIO_Port, ID2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(ID3_GPIO_Port, ID3_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(ID4_GPIO_Port, ID4_Pin, GPIO_PIN_RESET);
	// HAL_GPIO_WritePin(ID5_GPIO_Port, ID5_Pin, GPIO_PIN_RESET);
	// HAL_GPIO_WritePin(ID6_GPIO_Port, ID6_Pin, GPIO_PIN_RESET);
	for (uint8_t address = 0x30; address <= 0x45; address++) {
		if (address == 0x30) {
			HAL_GPIO_WritePin(ID1_GPIO_Port, ID1_Pin, GPIO_PIN_SET);
			VL6180X_Init(0x29 << 1, (0x29 << 1) | 0x01);
			VL6180x_ChangeAddress(address);
			VL6180X_Init(0x30 << 1, (0x30 << 1) | 0x01);
			VL6180X_SetScaling(3, 0x30 << 1, (0x30 << 1) | 0x01);
		}
		else if (address == 0x31) {
			HAL_GPIO_WritePin(ID2_GPIO_Port, ID2_Pin, GPIO_PIN_SET);
			VL6180X_Init(0x29 << 1, (0x29 << 1) | 0x01);
			VL6180x_ChangeAddress(address);
			VL6180X_Init(0x31 << 1, (0x31 << 1) | 0x01);
			VL6180X_SetScaling(3, 0x31 << 1, (0x31 << 1) | 0x01);
		}
		else if (address == 0x32) {
			HAL_GPIO_WritePin(ID3_GPIO_Port, ID3_Pin, GPIO_PIN_SET);
			VL6180X_Init(0x29 << 1, (0x29 << 1) | 0x01);
			VL6180x_ChangeAddress(address);
			VL6180X_Init(0x32 << 1, (0x32 << 1) | 0x01);
			VL6180X_SetScaling(3, 0x32 << 1, (0x32 << 1) | 0x01);
		}
		else if (address == 0x33) {
			HAL_GPIO_WritePin(ID4_GPIO_Port, ID4_Pin, GPIO_PIN_SET);
			VL6180X_Init(0x29 << 1, (0x29 << 1) | 0x01);
			VL6180x_ChangeAddress(address);
			VL6180X_Init(0x33 << 1, (0x33 << 1) | 0x01);
			VL6180X_SetScaling(3, 0x33 << 1, (0x33 << 1) | 0x01);
		}
		// else if (address == 0x34)
		// {
		//   HAL_GPIO_WritePin(ID5_GPIO_Port, ID5_Pin, GPIO_PIN_SET);
		//   VL6180X_Init(0x29 << 1, (0x29 << 1) | 0x01);
		//   VL6180x_ChangeAddress(address);
		//   VL6180X_Init(0x34 << 1, (0x34 << 1) | 0x01);
		//   VL6180X_SetScaling(3,0x34 << 1, (0x34 << 1) | 0x01);
		//
		// }
		// else if (address == 0x35)
		// {                 	 HAL_GPIO_WritePin(ID6_GPIO_Port, ID6_Pin, GPIO_PIN_SET);
		//   VL6180X_Init(0x29 << 1, (0x29 << 1) | 0x01);
		//   VL6180x_ChangeAddress(address);
		//   VL6180X_Init(0x35 << 1, (0x35 << 1) | 0x01);
		//   VL6180X_SetScaling(3,0x35 << 1, (0x35 << 1) | 0x01);
		// }
	}
}
