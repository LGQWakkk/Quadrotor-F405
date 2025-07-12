#pragma once

#include "system.h"
#include "spi.h"
#include <stdint.h>
#include <stdbool.h>

// Parameters: 
// x_scale = 2045.2250000276197, y_scale = 2048.8233945015827, z_scale = 2040.8016356111773
// x_bias = -5.241728980985401, y_bias = -5.4515755000735435, z_bias = 15.011007517159259
// ACC BIAS: [-5, -5, 15]
typedef struct{
    int16_t acc_bias[3];
    int16_t gyro_bias[3];
} icm42688_raw_bias_t;

typedef struct{
    float acc_ms2[3];
    float gyro_rads[3];
    uint32_t timestamp_ms;
} icm42688_data_t;

#define ICM42688_SPI_HANDLE     hspi2
#define BIT_READ_FLAG           0x80

#define SPI_CS_LOW() HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET)
#define SPI_CS_HIGH() HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET)

#ifdef BIT
#undef BIT
#endif

#define BIT(_idx) (1 << _idx)
#define REG_VAL(_setbits, _clearbits) \
    (reg_val_t) { .setbits = (_setbits), .clearbits = (_clearbits) }

#define DEVICE_ID 0x47

/* Bank0 Registers */
#define REG_DEVICE_CONFIG      0x11
#define REG_INT_CONFIG         0x14
#define REG_FIFO_CONFIG        0x16
#define REG_TEMP_DATA1         0x1D
#define REG_TEMP_DATA0         0x1E
#define REG_ACCEL_DATA_X1      0x1F
#define REG_GYRO_DATA_X1       0x25
#define REG_INT_STATUS         0x2D
#define REG_FIFO_COUNTH        0x2E
#define REG_FIFO_COUNTL        0x2F
#define REG_FIFO_DATA          0x30
#define REG_SIGNAL_PATH_RESET  0x4B
#define REG_INTF_CONFIG0       0x4C
#define REG_INTF_CONFIG1       0x4D
#define REG_PWR_MGMT0          0x4E
#define REG_GYRO_CONFIG0       0x4F
#define REG_ACCEL_CONFIG0      0x50
#define REG_GYRO_CONFIG1       0x51
#define REG_GYRO_ACCEL_CONFIG0 0x52
#define REG_ACCEL_CONFIG1      0x53
#define REG_TMST_CONFIG        0x54
#define REG_FIFO_CONFIG1       0x5F
#define REG_FIFO_CONFIG2       0x60
#define REG_FIFO_CONFIG3       0x61
#define REG_INT_CONFIG0        0x63
#define REG_INT_CONFIG1        0x64
#define REG_INT_SOURCE0        0x65
#define REG_SELF_TEST_CONFIG   0x70
#define REG_WHO_AM_I           0x75
#define REG_REG_BANK_SEL       0x76

/* Bank1 Registers */
#define REG_GYRO_CONFIG_STATIC2 0x0B
#define REG_INTF_CONFIG5        0x7B

/* Bank2 Registers */
#define REG_ACCEL_CONFIG_STATIC2 0x03

#define GYRO_RANGE_2000_DPS   REG_VAL(0, BIT(5) | BIT(6) | BIT(7))
#define GYRO_RANGE_1000_DPS   REG_VAL(BIT(5), BIT(6) | BIT(7))
#define GYRO_RANGE_500_DPS    REG_VAL(BIT(6), BIT(5) | BIT(7))
#define GYRO_RANGE_250_DPS    REG_VAL(BIT(5) | BIT(6), BIT(7))
#define GYRO_RANGE_125_DPS    REG_VAL(BIT(7), BIT(5) | BIT(6))
#define GYRO_RANGE_62_5_DPS   REG_VAL(BIT(5) | BIT(7), BIT(6))
#define GYRO_RANGE_31_25_DPS  REG_VAL(BIT(6) | BIT(7), BIT(5))
#define GYRO_RANGE_15_625_DPS REG_VAL(BIT(5) | BIT(6) | BIT(7), 0)

#define ACCEL_RANGE_16_G REG_VAL(0, BIT(5) | BIT(6) | BIT(7))
#define ACCEL_RANGE_8_G  REG_VAL(BIT(5), BIT(6) | BIT(7))
#define ACCEL_RANGE_4_G  REG_VAL(BIT(6), BIT(5) | BIT(7))
#define ACCEL_RANGE_2_G  REG_VAL(BIT(5) | BIT(6), BIT(7))

#define GYRO_ODR_32000 REG_VAL(BIT(0), BIT(1) | BIT(2) | BIT(3))
#define GYRO_ODR_16000 REG_VAL(BIT(1), BIT(1) | BIT(2) | BIT(3))
#define GYRO_ODR_8000  REG_VAL(BIT(0) | BIT(1), BIT(2) | BIT(3))
#define GYRO_ODR_4000  REG_VAL(BIT(2), BIT(0) | BIT(1) | BIT(3))
#define GYRO_ODR_2000  REG_VAL(BIT(0) | BIT(2), BIT(1) | BIT(3))
#define GYRO_ODR_1000  REG_VAL(BIT(1) | BIT(2), BIT(0) | BIT(3))
#define GYRO_ODR_500   REG_VAL(BIT(0) | BIT(1) | BIT(2) | BIT(3), 0)
#define GYRO_ODR_200   REG_VAL(BIT(0) | BIT(1) | BIT(2), BIT(3))
#define GYRO_ODR_100   REG_VAL(BIT(3), BIT(0) | BIT(1) | BIT(2))
#define GYRO_ODR_50    REG_VAL(BIT(0) | BIT(3), BIT(1) | BIT(2))
#define GYRO_ODR_25    REG_VAL(BIT(1) | BIT(3), BIT(0) | BIT(2))
#define GYRO_ODR_12_5  REG_VAL(BIT(0) | BIT(1) | BIT(3), BIT(2))

#define ACCEL_ODR_32000 REG_VAL(BIT(0), BIT(1) | BIT(2) | BIT(3))
#define ACCEL_ODR_16000 REG_VAL(BIT(1), BIT(1) | BIT(2) | BIT(3))
#define ACCEL_ODR_8000  REG_VAL(BIT(0) | BIT(1), BIT(2) | BIT(3))
#define ACCEL_ODR_4000  REG_VAL(BIT(2), BIT(0) | BIT(1) | BIT(3))
#define ACCEL_ODR_2000  REG_VAL(BIT(0) | BIT(2), BIT(1) | BIT(3))
#define ACCEL_ODR_1000  REG_VAL(BIT(1) | BIT(2), BIT(0) | BIT(3))
#define ACCEL_ODR_500   REG_VAL(BIT(0) | BIT(1) | BIT(2) | BIT(3), 0)
#define ACCEL_ODR_200   REG_VAL(BIT(0) | BIT(1) | BIT(2), BIT(3))
#define ACCEL_ODR_100   REG_VAL(BIT(3), BIT(0) | BIT(1) | BIT(2))
#define ACCEL_ODR_50    REG_VAL(BIT(0) | BIT(3), BIT(1) | BIT(2))
#define ACCEL_ODR_25    REG_VAL(BIT(1) | BIT(3), BIT(0) | BIT(2))
#define ACCEL_ODR_12_5  REG_VAL(BIT(0) | BIT(1) | BIT(3), BIT(2))

////////////////////////////////////////////////////////////////////////////////////////////////////
// 20250711 Wakkk

// Soft Reset
#define ICM426XX_RA_DEVICE_CONFIG                   0x17
#define DEVICE_CONFIG_SOFT_RESET_BIT                (1 << 0) // Soft reset bit

#define ICM426XX_RA_REG_BANK_SEL                    0x76
#define ICM426XX_BANK_SELECT0                       0x00
#define ICM426XX_BANK_SELECT1                       0x01
#define ICM426XX_BANK_SELECT2                       0x02
#define ICM426XX_BANK_SELECT3                       0x03
#define ICM426XX_BANK_SELECT4                       0x04

// Fix for stalls in gyro output. See https://github.com/ArduPilot/ardupilot/pull/25332
#define ICM426XX_INTF_CONFIG1                       0x4D
#define ICM426XX_INTF_CONFIG1_AFSR_MASK             0xC0
#define ICM426XX_INTF_CONFIG1_AFSR_DISABLE          0x40

#define ICM426XX_RA_PWR_MGMT0                       0x4E  // User Bank 0
#define ICM426XX_PWR_MGMT0_ACCEL_MODE_LN            (3 << 0)
#define ICM426XX_PWR_MGMT0_GYRO_MODE_LN             (3 << 2)
#define ICM426XX_PWR_MGMT0_GYRO_ACCEL_MODE_OFF      ((0 << 0) | (0 << 2))
#define ICM426XX_PWR_MGMT0_TEMP_DISABLE_OFF         (0 << 5)
#define ICM426XX_PWR_MGMT0_TEMP_DISABLE_ON          (1 << 5)

#define ICM426XX_RA_GYRO_CONFIG0                    0x4F
#define ICM426XX_RA_ACCEL_CONFIG0                   0x50

// --- Registers for gyro and acc Anti-Alias Filter ---------
#define ICM426XX_RA_GYRO_CONFIG_STATIC3             0x0C  // User Bank 1
#define ICM426XX_RA_GYRO_CONFIG_STATIC4             0x0D  // User Bank 1
#define ICM426XX_RA_GYRO_CONFIG_STATIC5             0x0E  // User Bank 1
#define ICM426XX_RA_ACCEL_CONFIG_STATIC2            0x03  // User Bank 2
#define ICM426XX_RA_ACCEL_CONFIG_STATIC3            0x04  // User Bank 2
#define ICM426XX_RA_ACCEL_CONFIG_STATIC4            0x05  // User Bank 2
// --- Register & setting for gyro and acc UI Filter --------
#define ICM426XX_RA_GYRO_ACCEL_CONFIG0              0x52  // User Bank 0
#define ICM426XX_ACCEL_UI_FILT_BW_LOW_LATENCY       (15 << 4)
#define ICM426XX_GYRO_UI_FILT_BW_LOW_LATENCY        (15 << 0)
// ----------------------------------------------------------

#define ICM426XX_RA_GYRO_DATA_X1                    0x25  // User Bank 0
#define ICM426XX_RA_ACCEL_DATA_X1                   0x1F  // User Bank 0

#define ICM426XX_RA_INT_CONFIG                      0x14  // User Bank 0
#define ICM426XX_INT1_MODE_PULSED                   (0 << 2)
#define ICM426XX_INT1_MODE_LATCHED                  (1 << 2)
#define ICM426XX_INT1_DRIVE_CIRCUIT_OD              (0 << 1)
#define ICM426XX_INT1_DRIVE_CIRCUIT_PP              (1 << 1)
#define ICM426XX_INT1_POLARITY_ACTIVE_LOW           (0 << 0)
#define ICM426XX_INT1_POLARITY_ACTIVE_HIGH          (1 << 0)

#define ICM426XX_RA_INT_CONFIG0                     0x63  // User Bank 0
#define ICM426XX_UI_DRDY_INT_CLEAR_ON_SBR           ((0 << 5) | (0 << 4))
#define ICM426XX_UI_DRDY_INT_CLEAR_ON_SBR_DUPLICATE ((0 << 5) | (1 << 4)) // duplicate setting in datasheet, Rev 1.8
#define ICM426XX_UI_DRDY_INT_CLEAR_ON_F1BR          ((1 << 5) | (0 << 4))
#define ICM426XX_UI_DRDY_INT_CLEAR_ON_SBR_AND_F1BR  ((1 << 5) | (1 << 4))

#define ICM426XX_RA_INT_CONFIG1                     0x64   // User Bank 0
#define ICM426XX_INT_ASYNC_RESET_BIT                4
#define ICM426XX_INT_TDEASSERT_DISABLE_BIT          5
#define ICM426XX_INT_TDEASSERT_ENABLED              (0 << ICM426XX_INT_TDEASSERT_DISABLE_BIT)
#define ICM426XX_INT_TDEASSERT_DISABLED             (1 << ICM426XX_INT_TDEASSERT_DISABLE_BIT)
#define ICM426XX_INT_TPULSE_DURATION_BIT            6
#define ICM426XX_INT_TPULSE_DURATION_100            (0 << ICM426XX_INT_TPULSE_DURATION_BIT)
#define ICM426XX_INT_TPULSE_DURATION_8              (1 << ICM426XX_INT_TPULSE_DURATION_BIT)

#define ICM426XX_RA_INT_SOURCE0                     0x65  // User Bank 0
#define ICM426XX_UI_DRDY_INT1_EN_DISABLED           (0 << 3)
#define ICM426XX_UI_DRDY_INT1_EN_ENABLED            (1 << 3)

// specific to CLKIN configuration
#define ICM426XX_INTF_CONFIG5                       0x7B  // User Bank 1
#define ICM426XX_INTF_CONFIG1_CLKIN                 (1 << 2)
#define ICM426XX_INTF_CONFIG5_PIN9_FUNCTION_MASK    (3 << 1)   // PIN9 mode config
#define ICM426XX_INTF_CONFIG5_PIN9_FUNCTION_CLKIN   (2 << 1)   // PIN9 as CLKIN

// GYRO 抗混叠滤波器配置

// // Configure gyro Anti-Alias Filter (see section 5.3 "ANTI-ALIAS FILTER")
// const mpuSensor_e gyroModel = gyro->mpuDetectionResult.sensor;
// aafConfig_t aafConfig = getGyroAafConfig(gyroModel, gyroConfig()->gyro_hardware_lpf);
// setUserBank(dev, ICM426XX_BANK_SELECT1);
// spiWriteReg(dev, ICM426XX_RA_GYRO_CONFIG_STATIC3, aafConfig.delt);
// spiWriteReg(dev, ICM426XX_RA_GYRO_CONFIG_STATIC4, aafConfig.deltSqr & 0xFF);
// spiWriteReg(dev, ICM426XX_RA_GYRO_CONFIG_STATIC5, (aafConfig.deltSqr >> 8) | (aafConfig.bitshift << 4));

// GYRO 抗混叠滤波器的带宽由下面这三个寄存器共同控制:
// ICM426XX_RA_GYRO_CONFIG_STATIC3 0x0C:
// [5:0] GYRO_AAF_DELT
// ICM426XX_RA_GYRO_CONFIG_STATIC4 0x0D:
// [7:0] GYRO_AAF_DELTSQR[7:0]
// ICM426XX_RA_GYRO_CONFIG_STATIC5 0x0E:
// [7:4] GYRO_AAF_BITSHIFT 
// [3:0] GYRO_AAF_DELTSQR[11:8]
// 硬件允许的带宽范围: 42Hz~3979Hz

typedef enum {
    AAF_CONFIG_258HZ = 0,
    AAF_CONFIG_536HZ,
    AAF_CONFIG_997HZ,
    AAF_CONFIG_1962HZ,
    AAF_CONFIG_COUNT
} aafConfig_e;

// GYRO 抗混叠滤波器配置结构体:
typedef struct aafConfig_s {
    uint8_t delt; // 6bits 1-63  
    uint16_t deltSqr; // 12bits (Square of the delt value for gyroscope)
    uint8_t bitshift; // 4bits (Bitshift value for gyroscope used in hardware implementation)
} aafConfig_t;

////////////////////////////////////////////////////////////////////////////////////////////////////

#define M_PI_F  3.1415926f
#define M_ONE_G 9.80665f

typedef struct{
    uint32_t sample_rate_hz;    /* sample rate in Hz */
    uint16_t dlpf_freq_hz;      /* internal low-pass filter cur-off freq in Hz */
    uint32_t acc_range_g;       /* accel measure range in g */
} accel_configure_t;

typedef struct{
    uint32_t sample_rate_hz;    /* sample rate in Hz */
    uint16_t dlpf_freq_hz;      /* internal low-pass filter cur-off freq in Hz */
    uint32_t gyro_range_dps;    /* gyro measure range in dps */
} gyro_configure_t;

#define GYRO_CONFIG                                   \
    {                                                 \
        1000,                   /* 1K sample rate */  \
            500,                /* 500Hz bandwidth */ \
            GYRO_RANGE_2000DPS, /* +-2000 deg/s */    \
    }

#define ACCEL_CONFIG                               \
    {                                              \
        1000,                /* 1K sample rate */  \
            500,             /* 500Hz bandwidth */ \
            ACCEL_RANGE_16G, /* +-16g */           \
    }

typedef struct {
    uint8_t setbits;
    uint8_t clearbits;
} reg_val_t;

void icm42688_spi_write_reg(uint8_t reg, uint8_t value);
uint8_t icm42688_spi_read_reg(uint8_t reg);
void icm42688_spi_read_buf(uint8_t reg, uint8_t *buf, uint16_t len);
void icm42688_rotate_to_frd(float* data, uint32_t dev_id);
void icm42688_modify_reg(uint8_t reg, reg_val_t reg_val);
bool gyro_set_range(uint32_t max_dps);
bool gyro_set_sample_rate(uint32_t frequency_hz);
bool gyro_set_dlpf_filter(uint16_t frequency_hz);
bool accel_set_range(uint32_t max_g);
bool accel_set_sample_rate(uint32_t frequency_hz);
bool accel_set_dlpf_filter(uint16_t frequency_hz);
void gyro_read_raw(int16_t gyro[3]);
void gyro_read_rad_s(float gyr[3]);
void accel_read_raw(int16_t accel[3]);
void accel_read_m_s2(float acc[3]);
bool accel_config(accel_configure_t *cfg);
bool gyro_config(gyro_configure_t *cfg);
bool icm42688_init(void);
int16_t int16_t_from_bytes(uint8_t bytes[]);
void icm42688_raw_data_debug(uint32_t system_tick_ms);
void gyro_corr_bias_raw(icm42688_raw_bias_t *bias, int16_t gyro[3]);
void acc_corr_bias_raw(icm42688_raw_bias_t *bias, int16_t accel[3]);
void icm42688_update(icm42688_data_t *data);

aafConfig_t icm42688_get_gyro_aaf_config(const aafConfig_e config);
void icm42688_turn_accgyro_off(void);
void icm42688_turn_accgyro_on(void);
