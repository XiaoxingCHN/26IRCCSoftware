//
// Created by LIMBO on 2026/4/24.
//
#include "vl53l0.h"
#include "stdio.h"

extern I2C_HandleTypeDef hi2c2;  // 假设使用 hi2c2，与 VL6180X 一致

uint16_t timeout_start_ms = 0;
uint16_t io_timeout = 0;
bool did_timeout = 0;

// Record the current time to check an upcoming timeout against
#define startTimeout() (timeout_start_ms = HAL_GetTick())

#define checkTimeoutExpired() (io_timeout > 0 && ((uint16_t)(HAL_GetTick() - timeout_start_ms) > io_timeout))

// Decode VCSEL (vertical cavity surface emitting laser) pulse period in PCLKs
// from register value
// based on VL53L0X_decode_vcsel_period()
#define decodeVcselPeriod(reg_val)      (((reg_val) + 1) << 1)

// Calculate macro period in *nanoseconds* from VCSEL period in PCLKs
// based on VL53L0X_calc_macro_period_ps()
// PLL_period_ps = 1655; macro_period_vclks = 2304
#define calcMacroPeriod(vcsel_period_pclks) ((((uint32_t)2304 * (vcsel_period_pclks) * 1655) + 500) / 1000)

void VL53L0X_setTimeout(uint16_t timeout_ms)
{
    io_timeout = timeout_ms;
}

void VL53L0X_WriteByte(uint8_t add, uint8_t reg, uint8_t data)
{
    HAL_I2C_Mem_Write(&hi2c2, (add << 1) | 0, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, 0xffff);
}

void VL53L0X_WriteByte_16Bit(uint8_t add, uint8_t reg, uint16_t data)
{
    uint8_t data2[2] = {0, 0};
    data2[0] = data >> 8;
    data2[1] = data;
    HAL_I2C_Mem_Write(&hi2c2, (add << 1) | 0, reg, I2C_MEMADD_SIZE_8BIT, data2, 2, 0xffff);
}

void VL53L0X_WriteByte_32Bit(uint8_t add, uint8_t reg, uint32_t data)
{
    uint8_t data2[4] = {0, 0, 0, 0};
    data2[0] = data >> 24;
    data2[1] = data >> 16;
    data2[2] = data >> 8;
    data2[3] = data;
    HAL_I2C_Mem_Write(&hi2c2, (add << 1) | 0, reg, I2C_MEMADD_SIZE_8BIT, data2, 4, 0xffff);
}

uint8_t VL53L0X_ReadByte(uint8_t add, uint8_t reg)
{
    uint8_t data = 0;
    HAL_I2C_Mem_Read(&hi2c2, (add << 1) | 1, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, 0xffff);
    return data;
}

uint16_t VL53L0X_ReadBytee_16Bit(uint8_t add, uint16_t reg)
{
    uint16_t data = 0;
    uint8_t data2[2];
    HAL_I2C_Mem_Read(&hi2c2, (add << 1) | 1, reg, I2C_MEMADD_SIZE_8BIT, data2, 2, 0xffff);
    data = data2[0];
    data = data << 8;
    data += data2[1];

    return data;
}

// Read an arbitrary number of bytes from the sensor, starting at the given
// register, into the given array
void VL53L0X_readMulti(uint8_t add, uint8_t reg, uint8_t *dst, uint8_t count)
{
    HAL_I2C_Mem_Read(&hi2c2, (add << 1) | 1, reg, I2C_MEMADD_SIZE_8BIT, dst, count, 0xffff);
}

// Write an arbitrary number of bytes from the given array to the sensor,
// starting at the given register
void VL53L0X_writeMulti(uint8_t add, uint8_t reg, uint8_t *src, uint8_t count)
{
    HAL_I2C_Mem_Write(&hi2c2, (add << 1) | 0, reg, I2C_MEMADD_SIZE_8BIT, src, count, 0xffff);
}

// Initialize sensor using sequence based on VL53L0X_DataInit(),
// VL53L0X_StaticInit(), and VL53L0X_PerformRefCalibration().
// This function does not perform reference SPAD calibration
// (VL53L0X_PerformRefSpadManagement()), since the API user manual says that it
// is performed by ST on the bare modules; it seems like that should work well
// enough unless a cover glass is added.
// If io_2v8 (optional) is true or not given, the sensor is configured for 2V8
// mode.

uint8_t ID_register;
uint8_t stop_variable; // read by init and used when starting measurement; is StopVariable field of VL53L0X_DevData_t structure in API
uint32_t measurement_timing_budget_us;

uint8_t VL53L0X_Init(uint8_t add, bool io_2v8)
{
    // check model ID register (value specified in datasheet)
    ID_register = VL53L0X_ReadByte(add, IDENTIFICATION_MODEL_ID); // VL53L0X_DEFAULT_I2C_ADDR1->C0
    // printf("\nID_register=%d\n", ID_register);
    if (ID_register != 0xEE)
        return false;

    // VL53L0X_DataInit() begin

    // sensor uses 1V8 mode for I/O by default; switch to 2V8 mode if necessary
    if (io_2v8)
    { // VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV->0x89
        VL53L0X_WriteByte(add, VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV, VL53L0X_ReadByte(add, VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV) | 0x01);
    }

    // "Set I2C standard mode"
    VL53L0X_WriteByte(add, 0x88, 0x00);

    VL53L0X_WriteByte(add, 0x80, 0x01);
    VL53L0X_WriteByte(add, 0xFF, 0x01);
    VL53L0X_WriteByte(add, 0x00, 0x00);
    stop_variable = VL53L0X_ReadByte(add, 0x91);
    // printf("\nstop_variable=%d\n", stop_variable);
    VL53L0X_WriteByte(add, 0x00, 0x01);
    VL53L0X_WriteByte(add, 0xFF, 0x00);
    VL53L0X_WriteByte(add, 0x80, 0x00);

    // disable SIGNAL_RATE_MSRC (bit 1) and SIGNAL_RATE_PRE_RANGE (bit 4) limit checks.MSRC_CONFIG_CONTROL->0x60
    VL53L0X_WriteByte(add, MSRC_CONFIG_CONTROL, VL53L0X_ReadByte(add, MSRC_CONFIG_CONTROL) | 0x12);

    // set final range signal rate limit to 0.25 MCPS (million counts per second)
    setSignalRateLimit(add, 0.25);

    VL53L0X_WriteByte(add, SYSTEM_SEQUENCE_CONFIG, 0xFF); // SYSTEM_SEQUENCE_CONFIG->  0x01

    // VL53L0X_DataInit() end

    // VL53L0X_StaticInit() begin
    uint8_t spad_count;
    bool spad_type_is_aperture;
    //
    if (!VL53L0X_getSpadInfo(add, &spad_count, &spad_type_is_aperture))
    {
        return false;
    }
    // The SPAD map (RefGoodSpadMap) is read by VL53L0X_get_info_from_device() in
    // the API, but the same data seems to be more easily readable from
    // GLOBAL_CONFIG_SPAD_ENABLES_REF_0 through _6, so read it from there
    uint8_t ref_spad_map[6];
    VL53L0X_readMulti(add, GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6); // GLOBAL_CONFIG_SPAD_ENABLES_REF_0->0xB0

    // -- VL53L0X_set_reference_spads() begin (assume NVM values are valid)

    VL53L0X_WriteByte(add, 0xFF, 0x01);
    VL53L0X_WriteByte(add, DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00); // DYNAMIC_SPAD_REF_EN_START_OFFSET->0x4F
    VL53L0X_WriteByte(add, DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C); // DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD->0x4E
    VL53L0X_WriteByte(add, 0xFF, 0x00);
    VL53L0X_WriteByte(add, GLOBAL_CONFIG_REF_EN_START_SELECT, 0xB4); // GLOBAL_CONFIG_REF_EN_START_SELECT->0xB6

    uint8_t first_spad_to_enable = spad_type_is_aperture ? 12 : 0; // 12 is the first aperture spad
    uint8_t spads_enabled = 0;

    for (uint8_t i = 0; i < 48; i++)
    {
        if (i < first_spad_to_enable || spads_enabled == spad_count)
        {
            // This bit is lower than the first one that should be enabled, or
            // (reference_spad_count) bits have already been enabled, so zero this bit
            ref_spad_map[i / 8] &= ~(1 << (i % 8));
        }
        else if ((ref_spad_map[i / 8] >> (i % 8)) & 0x1)
        {
            spads_enabled++;
        }
    }
    // GLOBAL_CONFIG_SPAD_ENABLES_REF_0->0xB0
    VL53L0X_writeMulti(add, GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6);
    // -- VL53L0X_set_reference_spads() end

    // -- VL53L0X_load_tuning_settings() begin
    // DefaultTuningSettings from vl53l0x_tuning.h

    VL53L0X_WriteByte(add, 0xFF, 0x01);
    VL53L0X_WriteByte(add, 0x00, 0x00);

    VL53L0X_WriteByte(add, 0xFF, 0x00);
    VL53L0X_WriteByte(add, 0x09, 0x00);
    VL53L0X_WriteByte(add, 0x10, 0x00);
    VL53L0X_WriteByte(add, 0x11, 0x00);

    VL53L0X_WriteByte(add, 0x24, 0x01);
    VL53L0X_WriteByte(add, 0x25, 0xFF);
    VL53L0X_WriteByte(add, 0x75, 0x00);

    VL53L0X_WriteByte(add, 0xFF, 0x01);
    VL53L0X_WriteByte(add, 0x4E, 0x2C);
    VL53L0X_WriteByte(add, 0x48, 0x00);
    VL53L0X_WriteByte(add, 0x30, 0x20);

    VL53L0X_WriteByte(add, 0xFF, 0x00);
    VL53L0X_WriteByte(add, 0x30, 0x09);
    VL53L0X_WriteByte(add, 0x54, 0x00);
    VL53L0X_WriteByte(add, 0x31, 0x04);
    VL53L0X_WriteByte(add, 0x32, 0x03);
    VL53L0X_WriteByte(add, 0x40, 0x83);
    VL53L0X_WriteByte(add, 0x46, 0x25);
    VL53L0X_WriteByte(add, 0x60, 0x00);
    VL53L0X_WriteByte(add, 0x27, 0x00);
    VL53L0X_WriteByte(add, 0x50, 0x06);
    VL53L0X_WriteByte(add, 0x51, 0x00);
    VL53L0X_WriteByte(add, 0x52, 0x96);
    VL53L0X_WriteByte(add, 0x56, 0x08);
    VL53L0X_WriteByte(add, 0x57, 0x30);
    VL53L0X_WriteByte(add, 0x61, 0x00);
    VL53L0X_WriteByte(add, 0x62, 0x00);
    VL53L0X_WriteByte(add, 0x64, 0x00);
    VL53L0X_WriteByte(add, 0x65, 0x00);
    VL53L0X_WriteByte(add, 0x66, 0xA0);

    VL53L0X_WriteByte(add, 0xFF, 0x01);
    VL53L0X_WriteByte(add, 0x22, 0x32);
    VL53L0X_WriteByte(add, 0x47, 0x14);
    VL53L0X_WriteByte(add, 0x49, 0xFF);
    VL53L0X_WriteByte(add, 0x4A, 0x00);

    VL53L0X_WriteByte(add, 0xFF, 0x00);
    VL53L0X_WriteByte(add, 0x7A, 0x0A);
    VL53L0X_WriteByte(add, 0x7B, 0x00);
    VL53L0X_WriteByte(add, 0x78, 0x21);

    VL53L0X_WriteByte(add, 0xFF, 0x01);
    VL53L0X_WriteByte(add, 0x23, 0x34);
    VL53L0X_WriteByte(add, 0x42, 0x00);
    VL53L0X_WriteByte(add, 0x44, 0xFF);
    VL53L0X_WriteByte(add, 0x45, 0x26);
    VL53L0X_WriteByte(add, 0x46, 0x05);
    VL53L0X_WriteByte(add, 0x40, 0x40);
    VL53L0X_WriteByte(add, 0x0E, 0x06);
    VL53L0X_WriteByte(add, 0x20, 0x1A);
    VL53L0X_WriteByte(add, 0x43, 0x40);

    VL53L0X_WriteByte(add, 0xFF, 0x00);
    VL53L0X_WriteByte(add, 0x34, 0x03);
    VL53L0X_WriteByte(add, 0x35, 0x44);

    VL53L0X_WriteByte(add, 0xFF, 0x01);
    VL53L0X_WriteByte(add, 0x31, 0x04);
    VL53L0X_WriteByte(add, 0x4B, 0x09);
    VL53L0X_WriteByte(add, 0x4C, 0x05);
    VL53L0X_WriteByte(add, 0x4D, 0x04);

    VL53L0X_WriteByte(add, 0xFF, 0x00);
    VL53L0X_WriteByte(add, 0x44, 0x00);
    VL53L0X_WriteByte(add, 0x45, 0x20);
    VL53L0X_WriteByte(add, 0x47, 0x08);
    VL53L0X_WriteByte(add, 0x48, 0x28);
    VL53L0X_WriteByte(add, 0x67, 0x00);
    VL53L0X_WriteByte(add, 0x70, 0x04);
    VL53L0X_WriteByte(add, 0x71, 0x01);
    VL53L0X_WriteByte(add, 0x72, 0xFE);
    VL53L0X_WriteByte(add, 0x76, 0x00);
    VL53L0X_WriteByte(add, 0x77, 0x00);

    VL53L0X_WriteByte(add, 0xFF, 0x01);
    VL53L0X_WriteByte(add, 0x0D, 0x01);

    VL53L0X_WriteByte(add, 0xFF, 0x00);
    VL53L0X_WriteByte(add, 0x80, 0x01);
    VL53L0X_WriteByte(add, 0x01, 0xF8);

    VL53L0X_WriteByte(add, 0xFF, 0x01);
    VL53L0X_WriteByte(add, 0x8E, 0x01);
    VL53L0X_WriteByte(add, 0x00, 0x01);
    VL53L0X_WriteByte(add, 0xFF, 0x00);
    VL53L0X_WriteByte(add, 0x80, 0x00);

    // -- VL53L0X_load_tuning_settings() end

    // "Set interrupt config to new sample ready"
    // -- VL53L0X_SetGpioConfig() begin

    VL53L0X_WriteByte(add, SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04); // SYSTEM_INTERRUPT_CONFIG_GPIO->0x0A
    // GPIO_HV_MUX_ACTIVE_HIGH->0x84
    VL53L0X_WriteByte(add, GPIO_HV_MUX_ACTIVE_HIGH, VL53L0X_ReadByte(add, GPIO_HV_MUX_ACTIVE_HIGH) & ~0x10); // active low
    // SYSTEM_INTERRUPT_CLEAR->0x0B
    VL53L0X_WriteByte(add, SYSTEM_INTERRUPT_CLEAR, 0x01);

    // -- VL53L0X_SetGpioConfig() end

    measurement_timing_budget_us = VL53L0X_getMeasurementTimingBudget(add);

    // "Disable MSRC and TCC by default"
    // MSRC = Minimum Signal Rate Check
    // TCC = Target CentreCheck
    // -- VL53L0X_SetSequenceStepEnable() begin
    // SYSTEM_SEQUENCE_CONFIG->0x01
    VL53L0X_WriteByte(add, SYSTEM_SEQUENCE_CONFIG, 0xE8);

    // -- VL53L0X_SetSequenceStepEnable() end

    // "Recalculate timing budget"
    VL53L0X_setMeasurementTimingBudget(add, measurement_timing_budget_us);

    // VL53L0X_StaticInit() end

    // VL53L0X_PerformRefCalibration() begin (VL53L0X_perform_ref_calibration())

    // -- VL53L0X_perform_vhv_calibration() begin
    // SYSRANGE_START->0x00
    VL53L0X_WriteByte(add, SYSRANGE_START, 0x01 | 0x40); // VL53L0X_REG_SYSRANGE_MODE_START_STOP

    startTimeout();
    // RESULT_INTERRUPT_STATUS->0x13
    while ((VL53L0X_ReadByte(add, RESULT_INTERRUPT_STATUS) & 0x07) == 0)
    {
        if (checkTimeoutExpired())
        {
            return false;
        }
    }
    // SYSTEM_INTERRUPT_CLEAR->0x0B
    VL53L0X_WriteByte(add, SYSTEM_INTERRUPT_CLEAR, 0x01);
    // SYSRANGE_START->0x00
    VL53L0X_WriteByte(add, SYSRANGE_START, 0x00);

    // -- VL53L0X_perform_vhv_calibration() end

    // -- VL53L0X_perform_phase_calibration() begin

    VL53L0X_WriteByte(add, SYSTEM_SEQUENCE_CONFIG, 0x02);
    if (!VL53L0X_performSingleRefCalibration(add, 0x00))
    {
        return false;
    }

    // -- VL53L0X_perform_phase_calibration() end

    // "restore the previous Sequence Config"
    VL53L0X_WriteByte(add, SYSTEM_SEQUENCE_CONFIG, 0xE8);

    // VL53L0X_PerformRefCalibration() end

    return true;
}

// Set the return signal rate limit check value in units of MCPS (mega counts
// per second). "This represents the amplitude of the signal reflected from the
// target and detected by the device"; setting this limit presumably determines
// the minimum measurement necessary for the sensor to report a valid reading.
// Setting a lower limit increases the potential range of the sensor but also
// seems to increase the likelihood of getting an inaccurate reading because of
// unwanted reflections from objects other than the intended target.
// Defaults to 0.25 MCPS as initialized by the ST API and this library.
uint8_t setSignalRateLimit(uint8_t add, float limit_Mcps)
{
    if (limit_Mcps < 0 || limit_Mcps > 511.99)
    {
        return 0;
    }

    // Q9.7 fixed point format (9 integer bits, 7 fractional bits)
    VL53L0X_WriteByte_16Bit(add, FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, limit_Mcps * (1 << 7));
    return 1;
}

// Set the measurement timing budget in microseconds, which is the time allowed
// for one measurement; the ST API and this library take care of splitting the
// timing budget among the sub-steps in the ranging sequence. A longer timing
// budget allows for more accurate measurements. Increasing the budget by a
// factor of N decreases the range measurement standard deviation by a factor of
// sqrt(N). Defaults to about 33 milliseconds; the minimum is 20 ms.
// based on VL53L0X_set_measurement_timing_budget_micro_seconds()
bool VL53L0X_setMeasurementTimingBudget(uint8_t add, uint32_t budget_us)
{
    // SequenceStepEnables enables;
    // SequenceStepTimeouts timeouts;

    uint16_t const StartOverhead = 1910;
    uint16_t const EndOverhead = 960;
    uint16_t const MsrcOverhead = 660;
    uint16_t const TccOverhead = 590;
    uint16_t const DssOverhead = 690;
    uint16_t const PreRangeOverhead = 660;
    uint16_t const FinalRangeOverhead = 550;

    uint32_t used_budget_us = StartOverhead + EndOverhead;

    // getSequenceStepEnables(&enables);
    uint8_t sequence_config = VL53L0X_ReadByte(add, SYSTEM_SEQUENCE_CONFIG); // SYSTEM_SEQUENCE_CONFIG->0x01

    uint8_t enables_tcc = (sequence_config >> 4) & 0x1;
    uint8_t enables_dss = (sequence_config >> 3) & 0x1;
    uint8_t enables_msrc = (sequence_config >> 2) & 0x1;
    uint8_t enables_pre_range = (sequence_config >> 6) & 0x1;
    uint8_t enables_final_range = (sequence_config >> 7) & 0x1;

    // getSequenceStepTimeouts(&enables, &timeouts);

    uint8_t timeouts_pre_range_vcsel_period_pclks = VL53L0X_getVcselPulsePeriod(add, 1); // VcselPeriodPreRange->1
    // MSRC_CONFIG_TIMEOUT_MACROP->0x46
    uint8_t timeouts_msrc_dss_tcc_mclks = VL53L0X_ReadByte(add, MSRC_CONFIG_TIMEOUT_MACROP) + 1;
    uint8_t timeouts_msrc_dss_tcc_us = VL53L0X_timeoutMclksToMicroseconds(timeouts_msrc_dss_tcc_mclks, timeouts_pre_range_vcsel_period_pclks);
    // PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI->0x51
    uint8_t timeouts_pre_range_mclks = VL53L0X_decodeTimeout(VL53L0X_ReadBytee_16Bit(add, PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI));
    uint8_t timeouts_pre_range_us = VL53L0X_timeoutMclksToMicroseconds(timeouts_pre_range_mclks, timeouts_pre_range_vcsel_period_pclks);

    uint8_t timeouts_final_range_vcsel_period_pclks = VL53L0X_getVcselPulsePeriod(add, 2); // VcselPeriodFinalRange->2
    // FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI->0x71
    uint8_t timeouts_final_range_mclks = VL53L0X_decodeTimeout(VL53L0X_ReadBytee_16Bit(add, FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI));

    if (enables_tcc)
    {
        used_budget_us += (timeouts_msrc_dss_tcc_us + TccOverhead);
    }

    if (enables_dss)
    {
        used_budget_us += 2 * (timeouts_msrc_dss_tcc_us + DssOverhead);
    }
    else if (enables_msrc)
    {
        used_budget_us += (timeouts_msrc_dss_tcc_us + MsrcOverhead);
    }

    if (enables_pre_range)
    {
        used_budget_us += (timeouts_pre_range_us + PreRangeOverhead);
    }

    if (enables_final_range)
    {
        used_budget_us += FinalRangeOverhead;

        // "Note that the final range timeout is determined by the timing
        // budget and the sum of all other timeouts within the sequence.
        // If there is no room for the final range timeout, then an error
        // will be set. Otherwise the remaining time will be applied to
        // the final range."

        if (used_budget_us > budget_us)
        {
            // "Requested timeout too big."
            return false;
        }

        uint32_t final_range_timeout_us = budget_us - used_budget_us;

        // set_sequence_step_timeout() begin
        // (SequenceStepId == VL53L0X_SEQUENCESTEP_FINAL_RANGE)

        // "For the final range timeout, the pre-range timeout
        //  must be added. To do this both final and pre-range
        //  timeouts must be expressed in macro periods MClks
        //  because they have different vcsel periods."

        uint32_t final_range_timeout_mclks = VL53L0X_timeoutMicrosecondsToMclks(final_range_timeout_us, timeouts_final_range_vcsel_period_pclks);

        if (enables_pre_range)
        {
            final_range_timeout_mclks += timeouts_pre_range_mclks;
        }
        // FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI->0x71
        VL53L0X_WriteByte_16Bit(add, FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, VL53L0X_encodeTimeout(final_range_timeout_mclks));

        // set_sequence_step_timeout() end

        measurement_timing_budget_us = budget_us; // store for internal reuse
    }
    return true;
}

// Get the measurement timing budget in microseconds
// based on VL53L0X_get_measurement_timing_budget_micro_seconds()
// in us
uint32_t VL53L0X_getMeasurementTimingBudget(uint8_t add)
{
    // SequenceStepEnables enables;
    // SequenceStepTimeouts timeouts;

    uint16_t const StartOverhead = 1910;
    uint16_t const EndOverhead = 960;
    uint16_t const MsrcOverhead = 660;
    uint16_t const TccOverhead = 590;
    uint16_t const DssOverhead = 690;
    uint16_t const PreRangeOverhead = 660;
    uint16_t const FinalRangeOverhead = 550;

    // "Start and end overhead times always present"
    uint32_t budget_us = StartOverhead + EndOverhead;

    // getSequenceStepEnables(&enables);
    uint8_t sequence_config = VL53L0X_ReadByte(add, SYSTEM_SEQUENCE_CONFIG); // SYSTEM_SEQUENCE_CONFIG->0x01

    uint8_t enables_tcc = (sequence_config >> 4) & 0x1;
    uint8_t enables_dss = (sequence_config >> 3) & 0x1;
    uint8_t enables_msrc = (sequence_config >> 2) & 0x1;
    uint8_t enables_pre_range = (sequence_config >> 6) & 0x1;
    uint8_t enables_final_range = (sequence_config >> 7) & 0x1;

    // getSequenceStepTimeouts(&enables, &timeouts);

    uint8_t timeouts_pre_range_vcsel_period_pclks = VL53L0X_getVcselPulsePeriod(add, 1); // VcselPeriodPreRange->1
    // MSRC_CONFIG_TIMEOUT_MACROP->0x46
    uint8_t timeouts_msrc_dss_tcc_mclks = VL53L0X_ReadByte(add, MSRC_CONFIG_TIMEOUT_MACROP) + 1;
    uint8_t timeouts_msrc_dss_tcc_us = VL53L0X_timeoutMclksToMicroseconds(timeouts_msrc_dss_tcc_mclks, timeouts_pre_range_vcsel_period_pclks);
    // PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI->0x51
    uint8_t timeouts_pre_range_mclks = VL53L0X_decodeTimeout(VL53L0X_ReadBytee_16Bit(add, PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI));
    uint8_t timeouts_pre_range_us = VL53L0X_timeoutMclksToMicroseconds(timeouts_pre_range_mclks, timeouts_pre_range_vcsel_period_pclks);

    uint8_t timeouts_final_range_vcsel_period_pclks = VL53L0X_getVcselPulsePeriod(add, 2); // VcselPeriodFinalRange->2
    // FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI->0x71
    uint8_t timeouts_final_range_mclks = VL53L0X_decodeTimeout(VL53L0X_ReadBytee_16Bit(add, FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI));

    if (enables_pre_range)
    {
        timeouts_final_range_mclks -= timeouts_pre_range_mclks;
    }
    uint8_t timeouts_final_range_us = VL53L0X_timeoutMclksToMicroseconds(timeouts_final_range_mclks, timeouts_final_range_vcsel_period_pclks);

    if (enables_tcc)
    {
        budget_us += (timeouts_msrc_dss_tcc_us + TccOverhead);
    }

    if (enables_dss)
    {
        budget_us += 2 * (timeouts_msrc_dss_tcc_us + DssOverhead);
    }
    else if (enables_msrc)
    {
        budget_us += (timeouts_msrc_dss_tcc_us + MsrcOverhead);
    }

    if (enables_pre_range)
    {
        budget_us += (timeouts_pre_range_us + PreRangeOverhead);
    }

    if (enables_final_range)
    {
        budget_us += (timeouts_final_range_us + FinalRangeOverhead);
    }

    measurement_timing_budget_us = budget_us; // store for internal reuse
    return budget_us;
}

// Get the VCSEL pulse period in PCLKs for the given period type.
// based on VL53L0X_get_vcsel_pulse_period()
uint8_t VL53L0X_getVcselPulsePeriod(uint8_t add, uint8_t type)
{
    if (type == 1) // VcselPeriodPreRange
    {
        // PRE_RANGE_CONFIG_VCSEL_PERIOD->0x50
        return decodeVcselPeriod(VL53L0X_ReadByte(add, PRE_RANGE_CONFIG_VCSEL_PERIOD));
    }
    else if (type == 2) // VcselPeriodFinalRange
    {
        // FINAL_RANGE_CONFIG_VCSEL_PERIOD->0x70
        return decodeVcselPeriod(VL53L0X_ReadByte(add, FINAL_RANGE_CONFIG_VCSEL_PERIOD));
    }
    else
    {
        return 255;
    }
}

// Start continuous ranging measurements. If period_ms (optional) is 0 or not
// given, continuous back-to-back mode is used (the sensor takes measurements as
// often as possible); otherwise, continuous timed mode is used, with the given
// inter-measurement period in milliseconds determining how often the sensor
// takes a measurement.
// based on VL53L0X_StartMeasurement()
void VL53L0X_startContinuous(uint8_t add, uint32_t period_ms)
{
    VL53L0X_WriteByte(add, 0x80, 0x01);
    VL53L0X_WriteByte(add, 0xFF, 0x01);
    VL53L0X_WriteByte(add, 0x00, 0x00);
    VL53L0X_WriteByte(add, 0x91, stop_variable);
    VL53L0X_WriteByte(add, 0x00, 0x01);
    VL53L0X_WriteByte(add, 0xFF, 0x00);
    VL53L0X_WriteByte(add, 0x80, 0x00);

    if (period_ms != 0)
    {
        // continuous timed mode

        // VL53L0X_SetInterMeasurementPeriodMilliSeconds() begin

        uint16_t osc_calibrate_val = VL53L0X_ReadBytee_16Bit(add, OSC_CALIBRATE_VAL);

        if (osc_calibrate_val != 0)
        {
            period_ms *= osc_calibrate_val;
        }

        VL53L0X_WriteByte_32Bit(add, SYSTEM_INTERMEASUREMENT_PERIOD, period_ms);

        // VL53L0X_SetInterMeasurementPeriodMilliSeconds() end

        VL53L0X_WriteByte(add, SYSRANGE_START, 0x04); // VL53L0X_REG_SYSRANGE_MODE_TIMED
    }
    else
    {
        // continuous back-to-back mode
        VL53L0X_WriteByte(add, SYSRANGE_START, 0x02); // VL53L0X_REG_SYSRANGE_MODE_BACKTOBACK
    }
}

// Returns a range reading in millimeters when continuous mode is active
// (readRangeSingleMillimeters() also calls this function after starting a
// single-shot range measurement)
uint16_t VL53L0X_readRangeContinuousMillimeters(uint8_t add)
{
    startTimeout();
    // uint16_t loop_count = 0;
    // const uint16_t MAX_LOOPS = 1000;
    uint16_t range;
    while ((VL53L0X_ReadByte(add, RESULT_INTERRUPT_STATUS) & 0x07) == 0)
    {
        if (checkTimeoutExpired())
        {
            did_timeout = true;
            return 65535;
        }
        // loop_count++;
        //        if (loop_count > MAX_LOOPS) {
        //            did_timeout = true;
        ////            printf("ERROR: Interrupt loop timeout for sensor 0x%02X\n", add);
        //            return 65535;
        //        }
    }
    // assumptions: Linearity Corrective Gain is 1000 (default);
    // fractional ranging is not enabled
    range = VL53L0X_ReadBytee_16Bit(add, RESULT_RANGE_STATUS + 10);

    VL53L0X_WriteByte(add, SYSTEM_INTERRUPT_CLEAR, 0x01);
    HAL_Delay(1);
    return range;
}

// Performs a single-shot range measurement and returns the reading in
// millimeters
// based on VL53L0X_PerformSingleRangingMeasurement()
uint16_t VL53L0X_readRangeSingleMillimeters(uint8_t add)
{
    VL53L0X_WriteByte(add, 0x80, 0x01);
    VL53L0X_WriteByte(add, 0xFF, 0x01);
    VL53L0X_WriteByte(add, 0x00, 0x00);
    VL53L0X_WriteByte(add, 0x91, stop_variable);
    VL53L0X_WriteByte(add, 0x00, 0x01);
    VL53L0X_WriteByte(add, 0xFF, 0x00);
    VL53L0X_WriteByte(add, 0x80, 0x00);

    VL53L0X_WriteByte(add, SYSRANGE_START, 0x01);

    // "Wait until start bit has been cleared"
    startTimeout();
    // uint16_t loop_count = 0;
    //    const uint16_t MAX_LOOPS = 1000;  // 额外保护，避免无限循环
    while (VL53L0X_ReadByte(add, SYSRANGE_START) & 0x01)
    {
        if (checkTimeoutExpired())
        {
            did_timeout = true;
            return 65535;
        }
        // loop_count++;
        //        if (loop_count > MAX_LOOPS) {
        //            did_timeout = true;
        ////            printf("ERROR: Loop timeout for sensor 0x%02X\n", add);
        //            return 65535;
        //        }

        HAL_Delay(1);
    }

    return VL53L0X_readRangeContinuousMillimeters(add);
}

// Get reference SPAD (single photon avalanche diode) count and type
// based on VL53L0X_get_info_from_device(),
// but only gets reference SPAD count and type
bool VL53L0X_getSpadInfo(uint8_t add, uint8_t *count, bool *type_is_aperture)
{
    uint8_t tmp;

    VL53L0X_WriteByte(add, 0x80, 0x01);
    VL53L0X_WriteByte(add, 0xFF, 0x01);
    VL53L0X_WriteByte(add, 0x00, 0x00);

    VL53L0X_WriteByte(add, 0xFF, 0x06);
    VL53L0X_WriteByte(add, 0x83, VL53L0X_ReadByte(add, 0x83) | 0x04);
    VL53L0X_WriteByte(add, 0xFF, 0x07);
    VL53L0X_WriteByte(add, 0x81, 0x01);

    VL53L0X_WriteByte(add, 0x80, 0x01);

    VL53L0X_WriteByte(add, 0x94, 0x6b);
    VL53L0X_WriteByte(add, 0x83, 0x00);
    //  startTimeout();
    while (VL53L0X_ReadByte(add, 0x83) == 0x00)
    {
        //    if (checkTimeoutExpired()) { return false; }
    }
    VL53L0X_WriteByte(add, 0x83, 0x01);
    tmp = VL53L0X_ReadByte(add, 0x92);

    *count = tmp & 0x7f;
    *type_is_aperture = (tmp >> 7) & 0x01;

    VL53L0X_WriteByte(add, 0x81, 0x00);
    VL53L0X_WriteByte(add, 0xFF, 0x06);
    VL53L0X_WriteByte(add, 0x83, VL53L0X_ReadByte(add, 0x83) & ~0x04);
    VL53L0X_WriteByte(add, 0xFF, 0x01);
    VL53L0X_WriteByte(add, 0x00, 0x01);

    VL53L0X_WriteByte(add, 0xFF, 0x00);
    VL53L0X_WriteByte(add, 0x80, 0x00);

    return true;
}

// Decode sequence step timeout in MCLKs from register value
// based on VL53L0X_decode_timeout()
// Note: the original function returned a uint32_t, but the return value is
// always stored in a uint16_t.
uint16_t VL53L0X_decodeTimeout(uint16_t reg_val)
{
    // format: "(LSByte * 2^MSByte) + 1"
    return (uint16_t)((reg_val & 0x00FF) <<
                      (uint16_t)((reg_val & 0xFF00) >> 8)) +
           1;
}

// Encode sequence step timeout register value from timeout in MCLKs
// based on VL53L0X_encode_timeout()
uint16_t VL53L0X_encodeTimeout(uint32_t timeout_mclks)
{
    // format: "(LSByte * 2^MSByte) + 1"

    uint32_t ls_byte = 0;
    uint16_t ms_byte = 0;

    if (timeout_mclks > 0)
    {
        ls_byte = timeout_mclks - 1;

        while ((ls_byte & 0xFFFFFF00) > 0)
        {
            ls_byte >>= 1;
            ms_byte++;
        }

        return (ms_byte << 8) | (ls_byte & 0xFF);
    }
    else
    {
        return 0;
    }
}

// Convert sequence step timeout from MCLKs to microseconds with given VCSEL period in PCLKs
// based on VL53L0X_calc_timeout_us()
uint32_t VL53L0X_timeoutMclksToMicroseconds(uint16_t timeout_period_mclks, uint8_t vcsel_period_pclks)
{
    uint32_t macro_period_ns = calcMacroPeriod(vcsel_period_pclks);

    return ((timeout_period_mclks * macro_period_ns) + 500) / 1000;
}

// Convert sequence step timeout from microseconds to MCLKs with given VCSEL period in PCLKs
// based on VL53L0X_calc_timeout_mclks()
uint32_t VL53L0X_timeoutMicrosecondsToMclks(uint32_t timeout_period_us, uint8_t vcsel_period_pclks)
{
    uint32_t macro_period_ns = calcMacroPeriod(vcsel_period_pclks);

    return (((timeout_period_us * 1000) + (macro_period_ns / 2)) / macro_period_ns);
}

// based on VL53L0X_perform_single_ref_calibration()
bool VL53L0X_performSingleRefCalibration(uint8_t add, uint8_t vhv_init_byte)
{
    // SYSRANGE_START->0x00
    VL53L0X_WriteByte(add, SYSRANGE_START, 0x01 | vhv_init_byte); // VL53L0X_REG_SYSRANGE_MODE_START_STOP

    startTimeout();
    // RESULT_INTERRUPT_STATUS->0x13
    while ((VL53L0X_ReadByte(add, RESULT_INTERRUPT_STATUS) & 0x07) == 0)
    {
        if (checkTimeoutExpired())
        {
            return false;
        }
    }
    // SYSTEM_INTERRUPT_CLEAR->0x0B
    VL53L0X_WriteByte(add, SYSTEM_INTERRUPT_CLEAR, 0x01);
    // SYSRANGE_START->0x00
    VL53L0X_WriteByte(add, SYSRANGE_START, 0x00);

    return true;
}

void Set_Address(uint8_t add, uint8_t new_addr)
{
    VL53L0X_WriteByte(add, I2C_SLAVE_DEVICE_ADDRESS, new_addr & 0x7F);
}

void multisensor_vl53l0_Init()
{
    HAL_GPIO_WritePin(ID5_GPIO_Port, ID5_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(ID6_GPIO_Port, ID6_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(ID7_GPIO_Port, ID7_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(ID8_GPIO_Port, ID8_Pin, GPIO_PIN_RESET);

    HAL_GPIO_WritePin(ID5_GPIO_Port, ID5_Pin, GPIO_PIN_SET);
    HAL_Delay(100);
    VL53L0X_Init(VL53L0X_DEFAULT_I2C_ADDR,true);
    Set_Address(VL53L0X_DEFAULT_I2C_ADDR,VL53L0X_DEFAULT_I2C_ADDR1);
    HAL_Delay(100);

    HAL_GPIO_WritePin(ID6_GPIO_Port, ID6_Pin, GPIO_PIN_SET);
    HAL_Delay(100);
    VL53L0X_Init(VL53L0X_DEFAULT_I2C_ADDR,true);
    Set_Address(VL53L0X_DEFAULT_I2C_ADDR,VL53L0X_DEFAULT_I2C_ADDR2);
    HAL_Delay(100);

    HAL_GPIO_WritePin(ID7_GPIO_Port, ID7_Pin, GPIO_PIN_SET);
    HAL_Delay(100);
    VL53L0X_Init(VL53L0X_DEFAULT_I2C_ADDR,true);
    Set_Address(VL53L0X_DEFAULT_I2C_ADDR,VL53L0X_DEFAULT_I2C_ADDR3);
    HAL_Delay(100);

    HAL_GPIO_WritePin(ID8_GPIO_Port, ID8_Pin, GPIO_PIN_SET);
    HAL_Delay(100);
    VL53L0X_Init(VL53L0X_DEFAULT_I2C_ADDR,true);
    Set_Address(VL53L0X_DEFAULT_I2C_ADDR,VL53L0X_DEFAULT_I2C_ADDR4);
    HAL_Delay(100);


}
