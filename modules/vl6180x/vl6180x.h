//
// Created by LIMBO on 2026/4/23.
//

#ifndef INC_26IRCCSOFTWARE_VL6180X_H
#define INC_26IRCCSOFTWARE_VL6180X_H

#include "main.h"
#include "i2c.h"
#include "tim.h"

typedef struct {
	uint8_t addr_w;
	uint8_t addr_r;
} TOF_050C_WRREG;

uint8_t VL6180X_Start_Range(uint8_t addr_write, uint8_t addr_read);

uint8_t VL6180X_Poll_Range(uint8_t addr_write, uint8_t addr_read);

uint8_t VL6180_Read_Range(uint8_t addr_write, uint8_t addr_read);

uint16_t VL6180X_ReadRangeSingleMillimeters(uint8_t addr_write, uint8_t addr_read);

void VL6180X_Clear_Interrupt(uint8_t addr_write, uint8_t addr_read);

uint8_t VL6180X_Init(uint8_t addr_write, uint8_t addr_read);

void VL6180X_ConfigureDefault(uint8_t addr_write, uint8_t addr_read);

void VL6180X_SetScaling(uint8_t new_scaling, uint8_t addr_write, uint8_t addr_read);

void VL6180X_SetTimeout(uint16_t timeout);
/**
 * @brief 同时读取多个VL6180X传感器的测距值，采用并行读取效率更高
 */
void VL6180X_ReadRangeMultiple(const TOF_050C_WRREG regs[],
                               uint16_t results[], uint8_t count);

void VL6180x_ChangeAddress(uint8_t new_address_7bit);

void AddressTest(void);

void multisensor_vl6180x(void);

#endif //INC_26IRCCSOFTWARE_VL6180X_H
