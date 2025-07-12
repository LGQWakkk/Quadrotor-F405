#include "spl06.h"
#include "stm32_i2c_driver.h"

static stm32_i2c_driver_t spl06_i2c;

// 校准系数
spl06_coeffs_t spl06_cal;
// 原始未补偿温度和压力
static int32_t spl06_pressure_raw = 0;
static int32_t spl06_temperature_raw = 0;
static float spl06_pressure = 0;
static float spl06_temperature = 0;

int8_t spl06_samples_to_cfg_reg_value(uint8_t sample_rate)
{
    switch(sample_rate){
        case 1: return 0;
        case 2: return 1;
        case 4: return 2;
        case 8: return 3;
        case 16: return 4;
        case 32: return 5;
        case 64: return 6;
        case 128: return 7;
        default: return -1; // invalid
    }
}

int32_t spl06_raw_value_scale_factor(uint8_t oversampling_rate)
{
    switch(oversampling_rate){
        case 1: return 524288;
        case 2: return 1572864;
        case 4: return 3670016;
        case 8: return 7864320;
        case 16: return 253952;
        case 32: return 516096;
        case 64: return 1040384;
        case 128: return 2088960;
        default: return -1; // invalid
    }
}

// 开始温度测量
bool spl06_start_temperature_measurement(void)
{
    return stm32_i2c_write_byte(&spl06_i2c, SPL06_MODE_AND_STATUS_REG, SPL06_MEAS_TEMPERATURE);
}

// 读取温度数据
bool spl06_read_temperature(void)
{
    uint8_t data[SPL06_TEMPERATURE_LEN];
    int32_t spl06_temperature;
    bool ret = stm32_i2c_read(&spl06_i2c, SPL06_TEMPERATURE_START_REG, data, SPL06_TEMPERATURE_LEN);
    spl06_temperature = (int32_t)((data[0] & 0x80 ? 0xFF000000 : 0) | (((uint32_t)(data[0])) << 16) | (((uint32_t)(data[1])) << 8) | ((uint32_t)data[2]));
    spl06_temperature_raw = spl06_temperature;
    return ret;
}

// 开始气压测量
bool spl06_start_pressure_measurement(void)
{
    return stm32_i2c_write_byte(&spl06_i2c, SPL06_MODE_AND_STATUS_REG, SPL06_MEAS_PRESSURE);
}

// 读取气压
bool spl06_read_pressure(void)
{
    uint8_t data[SPL06_PRESSURE_LEN];
    int32_t spl06_pressure;
    bool ret = stm32_i2c_read(&spl06_i2c, SPL06_PRESSURE_START_REG, data, SPL06_PRESSURE_LEN);
    spl06_pressure = (int32_t)((data[0] & 0x80 ? 0xFF000000 : 0) | (((uint32_t)(data[0])) << 16) | (((uint32_t)(data[1])) << 8) | ((uint32_t)data[2]));
    spl06_pressure_raw = spl06_pressure;
    return ret;
}

// Returns temperature in degrees centigrade
float spl06_compensate_temperature(int32_t temperature_raw)
{
    const float t_raw_sc = (float)temperature_raw / spl06_raw_value_scale_factor(SPL06_TEMPERATURE_OVERSAMPLING);
    const float temp_comp = (float)spl06_cal.c0 / 2 + t_raw_sc * spl06_cal.c1;
    return temp_comp;
}

// Returns pressure in Pascal
float spl06_compensate_pressure(int32_t pressure_raw, int32_t temperature_raw)
{
    const float p_raw_sc = (float)pressure_raw / spl06_raw_value_scale_factor(SPL06_PRESSURE_OVERSAMPLING);
    const float t_raw_sc = (float)temperature_raw / spl06_raw_value_scale_factor(SPL06_TEMPERATURE_OVERSAMPLING);

    const float pressure_cal = (float)spl06_cal.c00 + p_raw_sc * ((float)spl06_cal.c10 + p_raw_sc * ((float)spl06_cal.c20 + p_raw_sc * spl06_cal.c30));
    const float p_temp_comp = t_raw_sc * ((float)spl06_cal.c01 + p_raw_sc * ((float)spl06_cal.c11 + p_raw_sc * spl06_cal.c21));

    return pressure_cal + p_temp_comp;
}

bool spl06_calculate(float* pressure, float* temperature)
{
    if (pressure) {
        *pressure = spl06_compensate_pressure(spl06_pressure_raw, spl06_temperature_raw);
    }
    if (temperature) {
        *temperature = spl06_compensate_temperature(spl06_temperature_raw);
    }
    return true;
}

bool read_calibration_coefficients(void)
{
    uint8_t _status;
    _status = stm32_i2c_read_byte(&spl06_i2c, SPL06_MODE_AND_STATUS_REG);
    if(!(_status & SPL06_MEAS_CFG_COEFFS_RDY)){
        return false;   // error reading status or coefficients not ready
    }

    uint8_t caldata[SPL06_CALIB_COEFFS_LEN];

    stm32_i2c_read(&spl06_i2c, SPL06_CALIB_COEFFS_START, caldata, SPL06_CALIB_COEFFS_LEN);
    spl06_cal.c0 = (caldata[0] & 0x80 ? 0xF000 : 0) | ((uint16_t)caldata[0] << 4) | (((uint16_t)caldata[1] & 0xF0) >> 4);
    spl06_cal.c1 = ((caldata[1] & 0x8 ? 0xF000 : 0) | ((uint16_t)caldata[1] & 0x0F) << 8) | (uint16_t)caldata[2];
    spl06_cal.c00 = (caldata[3] & 0x80 ? 0xFFF00000 : 0) | ((uint32_t)caldata[3] << 12) | ((uint32_t)caldata[4] << 4) | (((uint32_t)caldata[5] & 0xF0) >> 4);
    spl06_cal.c10 = (caldata[5] & 0x8 ? 0xFFF00000 : 0) | (((uint32_t)caldata[5] & 0x0F) << 16) | ((uint32_t)caldata[6] << 8) | (uint32_t)caldata[7];
    spl06_cal.c01 = ((uint16_t)caldata[8] << 8) | ((uint16_t)caldata[9]);
    spl06_cal.c11 = ((uint16_t)caldata[10] << 8) | (uint16_t)caldata[11];
    spl06_cal.c20 = ((uint16_t)caldata[12] << 8) | (uint16_t)caldata[13];
    spl06_cal.c21 = ((uint16_t)caldata[14] << 8) | (uint16_t)caldata[15];
    spl06_cal.c30 = ((uint16_t)caldata[16] << 8) | (uint16_t)caldata[17];

    return true;
}

bool spl06_configure_measurements(void)
{
    uint8_t reg_value;

    reg_value = SPL06_TEMP_USE_EXT_SENSOR | spl06_samples_to_cfg_reg_value(SPL06_TEMPERATURE_OVERSAMPLING);
    stm32_i2c_write_byte(&spl06_i2c, SPL06_TEMPERATURE_CFG_REG, reg_value);

    reg_value = spl06_samples_to_cfg_reg_value(SPL06_PRESSURE_OVERSAMPLING);
    stm32_i2c_write_byte(&spl06_i2c, SPL06_PRESSURE_CFG_REG, reg_value);

    reg_value = 0;
    if (SPL06_TEMPERATURE_OVERSAMPLING > 8) {
        reg_value |= SPL06_TEMPERATURE_RESULT_BIT_SHIFT;
    }
    if (SPL06_PRESSURE_OVERSAMPLING > 8) {
        reg_value |= SPL06_PRESSURE_RESULT_BIT_SHIFT;
    }
    stm32_i2c_write_byte(&spl06_i2c, SPL06_INT_AND_FIFO_CFG_REG, reg_value);
    return true;
}

bool spl06Detect(void)
{
    slog("Finding SPL06...\r\n");
    uint8_t chipId;
    for (uint8_t retry = 0; retry < DETECTION_MAX_RETRY_COUNT; retry++) {
        HAL_Delay(100);
        chipId = stm32_i2c_read_byte(&spl06_i2c, SPL06_CHIP_ID_REG);
        if (chipId == SPL06_DEFAULT_CHIP_ID){
            slog("Found SPL06\r\n");
            return true;
        }
    };
    slog("SPL06 Not Found\r\n");
    return false;
}

bool spl06_init(void)
{
    // 初始化驱动链接
    spl06_i2c.addr = SPL06_I2C_ADDR<<1;
    spl06_i2c.handle = &SPL06_I2C_HANDLE;

    if(!spl06Detect()){
        return false;
    }

    slog("SPL06 Read Calibration Coefficients\r\n");
    if(!read_calibration_coefficients()){
        slog("Error: SPL06 Read Calibration Coefficients Failed\r\n");
        return false;
    }

    slog("SPL06 Configure Measurements\r\n");
    if(!spl06_configure_measurements()){
        slog("Error: SPL06 Configure Measurements Failed\r\n");
        return false;
    }
    return true;
}

void spl06_update(void)
{
    spl06_start_temperature_measurement();
    spl06_start_pressure_measurement();
    HAL_Delay(50);
    spl06_read_temperature();
    spl06_read_pressure();
    spl06_calculate(&spl06_pressure, &spl06_temperature);
    slog("SPL06 Pressure: %d, Temperature: %d\r\n", (int)(spl06_pressure*100.0f), (int)(spl06_temperature*100.0f));
}
