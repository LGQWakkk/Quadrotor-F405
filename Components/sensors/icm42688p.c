#include "icm42688p.h"

static float gyro_range_scale;
static float accel_range_scale;

icm42688_raw_bias_t raw_bias;

int16_t int16_t_from_bytes(uint8_t bytes[])
{
    union {
        uint8_t b[2];
        int16_t w;
    } u;
    u.b[1] = bytes[0];
    u.b[0] = bytes[1];
    return u.w;
}

void icm42688_spi_write_reg(uint8_t reg, uint8_t value)
{
    SPI_CS_LOW();
    HAL_SPI_Transmit(&ICM42688_SPI_HANDLE, &reg, 1, 1000);
    HAL_SPI_Transmit(&ICM42688_SPI_HANDLE, &value, 1, 1000);
    SPI_CS_HIGH();
}

uint8_t icm42688_spi_read_reg(uint8_t reg)
{
    uint8_t value;
    reg = reg | 0x80; // set the read bit
    SPI_CS_LOW();
    HAL_SPI_Transmit(&ICM42688_SPI_HANDLE, &reg, 1, 1000);
    HAL_SPI_Receive(&ICM42688_SPI_HANDLE, &value, 1, 1000);
    SPI_CS_HIGH();
    return value;
}

void icm42688_spi_read_buf(uint8_t reg, uint8_t *buf, uint16_t len)
{
    reg = reg | 0x80; // set the read bit
    SPI_CS_LOW();
    HAL_SPI_Transmit(&ICM42688_SPI_HANDLE, &reg, 1, 1000);
    HAL_SPI_Receive(&ICM42688_SPI_HANDLE, buf, len, 1000);
    SPI_CS_HIGH();
}

void icm42688_rotate_to_frd(float* data, uint32_t dev_id)
{
    (void)data;
    (void)dev_id;
}

void icm42688_modify_reg(uint8_t reg, reg_val_t reg_val)
{
    uint8_t value;
    value = icm42688_spi_read_reg(reg);
    value &= ~reg_val.clearbits;
    value |= reg_val.setbits;
    icm42688_spi_write_reg(reg, value);
}

bool gyro_set_range(uint32_t max_dps) //used
{
    reg_val_t reg_val;
    float lsb_per_dps;
    if (max_dps == 0) {
        max_dps = 2000;
    }
    if (max_dps <= 23) {
        reg_val = GYRO_RANGE_15_625_DPS;
        lsb_per_dps = 32768.0f / 15.625f;
    } else if (max_dps <= 47) {
        reg_val = GYRO_RANGE_31_25_DPS;
        lsb_per_dps = 32768.0f / 31.25f;
    } else if (max_dps <= 94) {
        reg_val = GYRO_RANGE_62_5_DPS;
        lsb_per_dps = 32768.0f / 62.5f;
    } else if (max_dps <= 188) {
        reg_val = GYRO_RANGE_125_DPS;
        lsb_per_dps = 32768.0f / 125.0f;
    } else if (max_dps <= 375) {
        reg_val = GYRO_RANGE_250_DPS;
        lsb_per_dps = 32768.0f / 250.0f;
    } else if (max_dps <= 750) {
        reg_val = GYRO_RANGE_500_DPS;
        lsb_per_dps = 32768.0f / 500.0f;
    } else if (max_dps <= 1500) {
        reg_val = GYRO_RANGE_1000_DPS;
        lsb_per_dps = 32768.0f / 1000.0f;
    } else if (max_dps <= 2000) {
        reg_val = GYRO_RANGE_2000_DPS;
        lsb_per_dps = 32768.0f / 2000.0f;
    } else {
        return false;
    }
    icm42688_modify_reg(REG_GYRO_CONFIG0, reg_val);
    gyro_range_scale = M_PI_F / 180.0f / lsb_per_dps;
    return true;
}

bool gyro_set_sample_rate(uint32_t frequency_hz) //used
{
    reg_val_t reg_val;
    if (frequency_hz <= 19) {
        reg_val = GYRO_ODR_12_5;
    } else if (frequency_hz <= 38) {
        reg_val = GYRO_ODR_25;
    } else if (frequency_hz <= 75) {
        reg_val = GYRO_ODR_50;
    } else if (frequency_hz <= 150) {
        reg_val = GYRO_ODR_100;
    } else if (frequency_hz <= 350) {
        reg_val = GYRO_ODR_200;
    } else if (frequency_hz <= 750) {
        reg_val = GYRO_ODR_500;
    } else if (frequency_hz <= 1500) {
        reg_val = GYRO_ODR_1000;
    } else if (frequency_hz <= 3000) {
        reg_val = GYRO_ODR_2000;
    } else if (frequency_hz <= 6000) {
        reg_val = GYRO_ODR_4000;
    } else if (frequency_hz <= 12000) {
        reg_val = GYRO_ODR_8000;
    } else if (frequency_hz <= 24000) {
        reg_val = GYRO_ODR_16000;
    } else if (frequency_hz <= 32000) {
        reg_val = GYRO_ODR_32000;
    } else {
        return false;
    }
    icm42688_modify_reg(REG_GYRO_CONFIG0, reg_val);
    return true;
}

bool gyro_set_dlpf_filter(uint16_t frequency_hz)
{
    /* do not enable lpf */
    return true;
}

bool accel_set_range(uint32_t max_g) //used
{
    reg_val_t reg_val;
    float lsb_per_g;
    if (max_g == 0) {
        max_g = 16;
    }
    if (max_g <= 3) {
        reg_val = ACCEL_RANGE_2_G;
        lsb_per_g = 32768.0f / 2.0f;
    } else if (max_g <= 6) {
        reg_val = ACCEL_RANGE_4_G;
        lsb_per_g = 32768.0f / 4.0f;
    } else if (max_g <= 12) {
        reg_val = ACCEL_RANGE_8_G;
        lsb_per_g = 32768.0f / 8.0f;
    } else if (max_g <= 16) {
        reg_val = ACCEL_RANGE_16_G;
        lsb_per_g = 32768.0f / 16.0f;
    } else {
        return false;
    }
    icm42688_modify_reg(REG_ACCEL_CONFIG0, reg_val);
    accel_range_scale = (M_ONE_G / lsb_per_g);
    return true;
}

bool accel_set_sample_rate(uint32_t frequency_hz) // OK
{
    reg_val_t reg_val;
    if (frequency_hz <= 19) {
        reg_val = ACCEL_ODR_12_5;
    } else if (frequency_hz <= 38) {
        reg_val = ACCEL_ODR_25;
    } else if (frequency_hz <= 75) {
        reg_val = ACCEL_ODR_50;
    } else if (frequency_hz <= 150) {
        reg_val = ACCEL_ODR_100;
    } else if (frequency_hz <= 350) {
        reg_val = ACCEL_ODR_200;
    } else if (frequency_hz <= 750) {
        reg_val = ACCEL_ODR_1000;
    } else if (frequency_hz <= 1500) {
        reg_val = ACCEL_ODR_2000;
    } else if (frequency_hz <= 3000) {
        reg_val = ACCEL_ODR_4000;
    } else if (frequency_hz <= 6000) {
        reg_val = ACCEL_ODR_8000;
    } else if (frequency_hz <= 12000) {
        reg_val = ACCEL_ODR_16000;
    } else if (frequency_hz <= 24000) {
        reg_val = ACCEL_ODR_32000;
    } else {
        return false;
    }
    icm42688_modify_reg(REG_ACCEL_CONFIG0, reg_val);
    return true;
}

bool accel_set_dlpf_filter(uint16_t frequency_hz)
{
    /* do not enable lpf */
    return true;
}

void gyro_read_raw(int16_t gyro[3])
{
    uint8_t raw[6];
    icm42688_spi_read_buf(REG_GYRO_DATA_X1, raw, 6);
    gyro[0] = int16_t_from_bytes(&raw[0]);
    gyro[1] = int16_t_from_bytes(&raw[2]);
    gyro[2] = int16_t_from_bytes(&raw[4]);
}

// 补偿GYRO原始偏置
void gyro_corr_bias_raw(icm42688_raw_bias_t *bias, int16_t gyro[3])
{
    gyro[0] -= bias->gyro_bias[0];
    gyro[1] -= bias->gyro_bias[1];
    gyro[2] -= bias->gyro_bias[2];
}

// 补偿ACC原始偏置
void acc_corr_bias_raw(icm42688_raw_bias_t *bias, int16_t accel[3])
{
    accel[0] -= bias->acc_bias[0];
    accel[1] -= bias->acc_bias[1];
    accel[2] -= bias->acc_bias[2];
}

// 外部调用接口
void gyro_read_rad_s(float gyr[3])
{
    int16_t gyr_raw[3];
    gyro_read_raw(gyr_raw);
    gyr[0] = gyro_range_scale * gyr_raw[0];
    gyr[1] = gyro_range_scale * gyr_raw[1];
    gyr[2] = gyro_range_scale * gyr_raw[2];
}

void accel_read_raw(int16_t accel[3])
{
    uint8_t raw[6];
    icm42688_spi_read_buf(REG_ACCEL_DATA_X1, raw, 6);
    accel[0] = int16_t_from_bytes(&raw[0]);
    accel[1] = int16_t_from_bytes(&raw[2]);
    accel[2] = int16_t_from_bytes(&raw[4]);
}

// 外部调用接口
void accel_read_m_s2(float acc[3])
{
    int16_t acc_raw[3];
    accel_read_raw(acc_raw);
    acc[0] = accel_range_scale * acc_raw[0];
    acc[1] = accel_range_scale * acc_raw[1];
    acc[2] = accel_range_scale * acc_raw[2];
}

bool accel_config(accel_configure_t *cfg)
{
    accel_set_range(cfg->acc_range_g);
    accel_set_sample_rate(cfg->sample_rate_hz);
    accel_set_dlpf_filter(cfg->dlpf_freq_hz);
    return true;
}

bool gyro_config(gyro_configure_t *cfg)
{
    gyro_set_range(cfg->gyro_range_dps);
    gyro_set_sample_rate(cfg->sample_rate_hz);
    gyro_set_dlpf_filter(cfg->dlpf_freq_hz);
    return true;
}

// 执行单次更新 最新数据存入data
void icm42688_update(icm42688_data_t *data)
{
    int16_t acc_raw[3];
    int16_t gyro_raw[3];
    data->timestamp_ms = get_system_1khz_tick();
    accel_read_raw(acc_raw);
    gyro_read_raw(gyro_raw);
    acc_corr_bias_raw(&raw_bias, acc_raw);
    gyro_corr_bias_raw(&raw_bias, gyro_raw);

    // 这里可以对坐标系进行变换 对于ICM42688P:
    // ACC Y轴取反
    // GYRO Z轴取反
    acc_raw[1] = -acc_raw[1];
    gyro_raw[0] = -gyro_raw[0];
    gyro_raw[2] = -gyro_raw[2];

    data->acc_ms2[0] = accel_range_scale * acc_raw[0];
    data->acc_ms2[1] = accel_range_scale * acc_raw[1];
    data->acc_ms2[2] = accel_range_scale * acc_raw[2];
    data->gyro_rads[0] = gyro_range_scale * gyro_raw[0];
    data->gyro_rads[1] = gyro_range_scale * gyro_raw[1];
    data->gyro_rads[2] = gyro_range_scale * gyro_raw[2];
}

// 实际常用的搭配: 此表和数据手册相对应
// Possible gyro Anti-Alias Filter (AAF) cutoffs for ICM-42688P
aafConfig_t aafLUT42688[AAF_CONFIG_COUNT] = {  // see table in section 5.3
    [AAF_CONFIG_258HZ]  = {  6,   36, 10 },
    [AAF_CONFIG_536HZ]  = { 12,  144,  8 },
    [AAF_CONFIG_997HZ]  = { 21,  440,  6 },
    [AAF_CONFIG_1962HZ] = { 37, 1376,  4 },
};

// 根据配置枚举返回配置组合数值
aafConfig_t icm42688_get_gyro_aaf_config(const aafConfig_e config)
{
    switch (config) {
    case AAF_CONFIG_258HZ:
        return aafLUT42688[AAF_CONFIG_258HZ];
    case AAF_CONFIG_536HZ:
        return aafLUT42688[AAF_CONFIG_536HZ];
    case AAF_CONFIG_997HZ:
        return aafLUT42688[AAF_CONFIG_997HZ];
    case AAF_CONFIG_1962HZ:
        return aafLUT42688[AAF_CONFIG_1962HZ];
    default:
        return aafLUT42688[AAF_CONFIG_258HZ];
    }
}

void icm42688_turn_accgyro_off(void)
{
    icm42688_spi_write_reg(ICM426XX_RA_PWR_MGMT0, ICM426XX_PWR_MGMT0_GYRO_ACCEL_MODE_OFF);
}

// Turn on gyro and acc on in Low Noise mode
void icm42688_turn_accgyro_on(void)
{
    icm42688_spi_write_reg(ICM426XX_RA_PWR_MGMT0, ICM426XX_PWR_MGMT0_TEMP_DISABLE_OFF | ICM426XX_PWR_MGMT0_ACCEL_MODE_LN | ICM426XX_PWR_MGMT0_GYRO_MODE_LN);
    HAL_Delay(30);
}

// SPI MODE3
bool icm42688_init(void)
{
    bool found = false;
    for(uint8_t i=0; i<5; i++){
        uint8_t dev_id = icm42688_spi_read_reg(REG_WHO_AM_I);
        if(dev_id != DEVICE_ID){
            slog("ICM42688P Wrong Device id:0x%x\n", dev_id);
        }else{
            slog("ICM42688P Device id:0x%x\n", dev_id);
            found = true;
            break;
        }
        HAL_Delay(20);
    }
    if(!found)return false;

    // 原始配置: 
    // /* select bank0 */
    // icm42688_spi_write_reg(REG_REG_BANK_SEL, 0);
    // /* temperature sensor enabled (note that if it's disabled there will be a bias for gyro), 
    //    gyro and accel work in low noise (LN) mode */
    // icm42688_spi_write_reg(REG_PWR_MGMT0, 0x0F);
    // /* gyro need 30ms startup time */
    // HAL_Delay(30);
    // /* gyro config */
    // gyro_set_range(2000);
    // gyro_set_sample_rate(1000);
    // /* accel config */
    // accel_set_range(16);
    // accel_set_sample_rate(1000);

    // // UI 滤波器配置(不使用此配置)
    // icm42688_spi_write_reg(REG_GYRO_ACCEL_CONFIG0, 0x00);/* 0 BW (ODR/2) for gyro and accel */
    // icm42688_spi_write_reg(REG_GYRO_ACCEL_CONFIG0, 0x11); // 0001 0001 BW=250Hz
    // icm42688_spi_write_reg(REG_GYRO_ACCEL_CONFIG0, 0x41); // 0100 0001 ACCBW=100Hz GYROBW=250Hz

    // // 抗混叠滤波器配置(不使用抗混叠滤波器)
    // icm42688_spi_write_reg(REG_REG_BANK_SEL, 1);/* select bank1 */
    // icm42688_spi_write_reg(REG_GYRO_CONFIG_STATIC2, 0x03);/* disable gyro internal anti-aliasing filter and notch filter */
    // icm42688_spi_write_reg(REG_REG_BANK_SEL, 2);/* select bank2 */
    // icm42688_spi_write_reg(REG_ACCEL_CONFIG_STATIC2, 0x01);/* disable accel internal anti-aliasing filter */
    // icm42688_spi_write_reg(REG_REG_BANK_SEL, 0);/* select bank0 */

////////////////////////////////////// 20250711 更新配置 //////////////////////////////////////
    // Turn off ACC and GYRO so they can be configured
    // See section 12.9 in ICM-42688-P datasheet v1.7
    icm42688_spi_write_reg(REG_REG_BANK_SEL, 0);
    icm42688_turn_accgyro_off();
    // 抗混叠滤波器配置(使用抗混叠滤波器-20250711)
    // Configure gyro Anti-Alias Filter (see section 5.3 "ANTI-ALIAS FILTER")
    aafConfig_t aafConfig = icm42688_get_gyro_aaf_config(AAF_CONFIG_258HZ);
    icm42688_spi_write_reg(REG_REG_BANK_SEL, 1); /* select bank1 */
    icm42688_spi_write_reg(ICM426XX_RA_GYRO_CONFIG_STATIC3, aafConfig.delt);
    icm42688_spi_write_reg(ICM426XX_RA_GYRO_CONFIG_STATIC4, aafConfig.deltSqr & 0xFF);
    icm42688_spi_write_reg(ICM426XX_RA_GYRO_CONFIG_STATIC5, (aafConfig.deltSqr >> 8) | (aafConfig.bitshift << 4));
    // Configure acc Anti-Alias Filter for 1kHz sample rate
    aafConfig = icm42688_get_gyro_aaf_config(AAF_CONFIG_258HZ);
    icm42688_spi_write_reg(REG_REG_BANK_SEL, 2); /* select bank2 */
    icm42688_spi_write_reg(ICM426XX_RA_ACCEL_CONFIG_STATIC2, aafConfig.delt << 1);
    icm42688_spi_write_reg(ICM426XX_RA_ACCEL_CONFIG_STATIC3, aafConfig.deltSqr & 0xFF);
    icm42688_spi_write_reg(ICM426XX_RA_ACCEL_CONFIG_STATIC4, (aafConfig.deltSqr >> 8) | (aafConfig.bitshift << 4));
    // Configure gyro and acc UI Filters
    icm42688_spi_write_reg(REG_REG_BANK_SEL, 0); /* select bank0 */
    icm42688_spi_write_reg(ICM426XX_RA_GYRO_ACCEL_CONFIG0, ICM426XX_ACCEL_UI_FILT_BW_LOW_LATENCY | ICM426XX_GYRO_UI_FILT_BW_LOW_LATENCY);
    
    // Configure interrupt pin 配置中断以及中断引脚
    // icm42688_spi_write_reg(ICM426XX_RA_INT_CONFIG, ICM426XX_INT1_MODE_PULSED | ICM426XX_INT1_DRIVE_CIRCUIT_PP | ICM426XX_INT1_POLARITY_ACTIVE_HIGH);
    // icm42688_spi_write_reg(ICM426XX_RA_INT_CONFIG0, ICM426XX_UI_DRDY_INT_CLEAR_ON_SBR);
    // icm42688_spi_write_reg(ICM426XX_RA_INT_SOURCE0, ICM426XX_UI_DRDY_INT1_EN_ENABLED);
    // uint8_t intConfig1Value = spiReadRegMsk(dev, ICM426XX_RA_INT_CONFIG1);
    // // Datasheet says: "User should change setting to 0 from default setting of 1, for proper INT1 and INT2 pin operation"
    // intConfig1Value &= ~(1 << ICM426XX_INT_ASYNC_RESET_BIT);
    // intConfig1Value |= (ICM426XX_INT_TPULSE_DURATION_8 | ICM426XX_INT_TDEASSERT_DISABLED);
    // icm42688_spi_write_reg(ICM426XX_RA_INT_CONFIG1, intConfig1Value);

    // Disable AFSR to prevent stalls in gyro output
    uint8_t intfConfig1Value = icm42688_spi_read_reg(ICM426XX_INTF_CONFIG1);
    intfConfig1Value &= ~ICM426XX_INTF_CONFIG1_AFSR_MASK;
    intfConfig1Value |= ICM426XX_INTF_CONFIG1_AFSR_DISABLE;
    icm42688_spi_write_reg(ICM426XX_INTF_CONFIG1, intfConfig1Value);

    // Turn on gyro and acc on again so ODR and FSR can be configured
    icm42688_turn_accgyro_on();
    /* gyro config */
    gyro_set_range(2000);
    gyro_set_sample_rate(1000);
    HAL_Delay(15);
    /* accel config */
    accel_set_range(16);
    accel_set_sample_rate(1000);
    HAL_Delay(15);
////////////////////////////////////// 20250711 更新配置 //////////////////////////////////////

    // 初始化默认BIAS
    raw_bias.acc_bias[0] =   -5;
    raw_bias.acc_bias[1] =   -5;
    raw_bias.acc_bias[2] =   15;
    raw_bias.gyro_bias[0] =   6;
    raw_bias.gyro_bias[1] = -13;
    raw_bias.gyro_bias[2] =  -4;

    return true;
}

// DEBUG UART 发送原始数据
// 直接使用UART DMA发送 注意DMA冲突
// 数据包格式: [0xAA] [System Tick(ms)] [ACC RAW 6bytes] [GYRO RAW 6bytes] [0x55]
// 1000000baudrate: 单个数据包发送时间179us
// 注意此函数会执行SPI读取!
uint8_t raw_data_buf[18];
void icm42688_raw_data_debug(uint32_t system_tick_ms)
{
    raw_data_buf[0] = 0xAA;
    memcpy(&raw_data_buf[1], &system_tick_ms, 4);
    icm42688_spi_read_buf(REG_ACCEL_DATA_X1, &raw_data_buf[5], 6);
    icm42688_spi_read_buf(REG_GYRO_DATA_X1, &raw_data_buf[11], 6);
    raw_data_buf[17] = 0x55;
    // DMA SEND
    debug_uart_tx_dma((uint32_t)raw_data_buf, 18);
}

// gyro_read:
// gyro_read_rad_s
// icm42688_rotate_to_frd

// accel_read:
// accel_read_m_s2
// icm42688_rotate_to_frd
