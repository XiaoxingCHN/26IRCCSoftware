//
// Created by LIMBO on 2026/4/24.
//

#ifndef INC_26IRCCSOFTWARE_VL53L0_H
#define INC_26IRCCSOFTWARE_VL53L0_H

#include "main.h"
#include <stdbool.h>

// 宏定义
#define VL53L0X_DEFAULT_I2C_ADDR 0x29

#define VL53L0X_DEFAULT_I2C_ADDR1 0x34  ///< The fixed I2C address
#define VL53L0X_DEFAULT_I2C_ADDR2 0x35
#define VL53L0X_DEFAULT_I2C_ADDR3 0x36
#define VL53L0X_DEFAULT_I2C_ADDR4 0x37




#define SYSRANGE_START                              0x00

#define SYSTEM_THRESH_HIGH                          0x0C
#define SYSTEM_THRESH_LOW                           0x0E

#define SYSTEM_SEQUENCE_CONFIG                      0x01
#define SYSTEM_RANGE_CONFIG                         0x09
#define SYSTEM_INTERMEASUREMENT_PERIOD              0x04

#define SYSTEM_INTERRUPT_CONFIG_GPIO                0x0A

#define GPIO_HV_MUX_ACTIVE_HIGH                     0x84

#define SYSTEM_INTERRUPT_CLEAR                      0x0B

#define RESULT_INTERRUPT_STATUS                     0x13
#define RESULT_RANGE_STATUS                         0x14

#define RESULT_CORE_AMBIENT_WINDOW_EVENTS_RTN       0xBC
#define RESULT_CORE_RANGING_TOTAL_EVENTS_RTN        0xC0
#define RESULT_CORE_AMBIENT_WINDOW_EVENTS_REF       0xD0
#define RESULT_CORE_RANGING_TOTAL_EVENTS_REF        0xD4
#define RESULT_PEAK_SIGNAL_RATE_REF                 0xB6

#define ALGO_PART_TO_PART_RANGE_OFFSET_MM           0x28

#define I2C_SLAVE_DEVICE_ADDRESS                    0x8A

#define MSRC_CONFIG_CONTROL                         0x60

#define PRE_RANGE_CONFIG_MIN_SNR                    0x27
#define PRE_RANGE_CONFIG_VALID_PHASE_LOW            0x56
#define PRE_RANGE_CONFIG_VALID_PHASE_HIGH           0x57
#define PRE_RANGE_MIN_COUNT_RATE_RTN_LIMIT          0x64

#define FINAL_RANGE_CONFIG_MIN_SNR                  0x67
#define FINAL_RANGE_CONFIG_VALID_PHASE_LOW          0x47
#define FINAL_RANGE_CONFIG_VALID_PHASE_HIGH         0x48
#define FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT 0x44

#define PRE_RANGE_CONFIG_SIGMA_THRESH_HI            0x61
#define PRE_RANGE_CONFIG_SIGMA_THRESH_LO            0x62

#define PRE_RANGE_CONFIG_VCSEL_PERIOD               0x50
#define PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI          0x51
#define PRE_RANGE_CONFIG_TIMEOUT_MACROP_LO          0x52

#define SYSTEM_HISTOGRAM_BIN                        0x81
#define HISTOGRAM_CONFIG_INITIAL_PHASE_SELECT       0x33
#define HISTOGRAM_CONFIG_READOUT_CTRL               0x55

#define FINAL_RANGE_CONFIG_VCSEL_PERIOD             0x70
#define FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI        0x71
#define FINAL_RANGE_CONFIG_TIMEOUT_MACROP_LO        0x72
#define CROSSTALK_COMPENSATION_PEAK_RATE_MCPS       0x20

#define MSRC_CONFIG_TIMEOUT_MACROP                  0x46

#define SOFT_RESET_GO2_SOFT_RESET_N                 0xBF
#define IDENTIFICATION_MODEL_ID                     0xC0
#define IDENTIFICATION_REVISION_ID                  0xC2

#define OSC_CALIBRATE_VAL                           0xF8

#define GLOBAL_CONFIG_VCSEL_WIDTH                   0x32
#define GLOBAL_CONFIG_SPAD_ENABLES_REF_0            0xB0
#define GLOBAL_CONFIG_SPAD_ENABLES_REF_1            0xB1
#define GLOBAL_CONFIG_SPAD_ENABLES_REF_2            0xB2
#define GLOBAL_CONFIG_SPAD_ENABLES_REF_3            0xB3
#define GLOBAL_CONFIG_SPAD_ENABLES_REF_4            0xB4
#define GLOBAL_CONFIG_SPAD_ENABLES_REF_5            0xB5

#define GLOBAL_CONFIG_REF_EN_START_SELECT           0xB6
#define DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD         0x4E
#define DYNAMIC_SPAD_REF_EN_START_OFFSET            0x4F
#define POWER_MANAGEMENT_GO1_POWER_FORCE            0x80

#define VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV           0x89

#define ALGO_PHASECAL_LIM                           0x30
#define ALGO_PHASECAL_CONFIG_TIMEOUT                0x30

// 函数声明
void VL53L0X_setTimeout(uint16_t timeout_ms);

// 写入函数
void VL53L0X_WriteByte(uint8_t add, uint8_t reg, uint8_t data);
void VL53L0X_WriteByte_16Bit(uint8_t add, uint8_t reg, uint16_t data);
void VL53L0X_WriteByte_32Bit(uint8_t add, uint8_t reg, uint32_t data);

// 读取函数
uint8_t VL53L0X_ReadByte(uint8_t add, uint8_t reg);
uint16_t VL53L0X_ReadBytee_16Bit(uint8_t add, uint16_t reg);
void VL53L0X_readMulti(uint8_t add, uint8_t reg, uint8_t *dst, uint8_t count);
void VL53L0X_writeMulti(uint8_t add, uint8_t reg, uint8_t *src, uint8_t count);

// 初始化
uint8_t VL53L0X_Init(uint8_t add, bool io_2v8);

// 配置
uint8_t setSignalRateLimit(uint8_t add, float limit_Mcps);
bool VL53L0X_setMeasurementTimingBudget(uint8_t add, uint32_t budget_us);
uint32_t VL53L0X_getMeasurementTimingBudget(uint8_t add);
uint8_t VL53L0X_getVcselPulsePeriod(uint8_t add, uint8_t type);

// 测量
void VL53L0X_startContinuous(uint8_t add, uint32_t period_ms);
uint16_t VL53L0X_readRangeContinuousMillimeters(uint8_t add);
uint16_t VL53L0X_readRangeSingleMillimeters(uint8_t add);

// 辅助函数
bool VL53L0X_getSpadInfo(uint8_t add, uint8_t *count, bool *type_is_aperture);
uint16_t VL53L0X_decodeTimeout(uint16_t reg_val);
uint16_t VL53L0X_encodeTimeout(uint32_t timeout_mclks);
uint32_t VL53L0X_timeoutMclksToMicroseconds(uint16_t timeout_period_mclks, uint8_t vcsel_period_pclks);
uint32_t VL53L0X_timeoutMicrosecondsToMclks(uint32_t timeout_period_us, uint8_t vcsel_period_pclks);
bool VL53L0X_performSingleRefCalibration(uint8_t add, uint8_t vhv_init_byte);

void Set_Address(uint8_t add, uint8_t new_addr);
void multisensor_vl53l0_Init(void);
#endif // INC_26IRCCSOFTWARE_VL53L0_H
