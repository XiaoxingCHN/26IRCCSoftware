#include "bsp_adc.h"
static uint16_t adc_val;
float vbus;

void ADC_VBAT_INIT() {
	HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t *) &adc_val, 1);
}

float VBATVAL_GET() {
	vbus = ((float) adc_val * 3.3f / 65535) * 11.0f;
	return vbus;
}


