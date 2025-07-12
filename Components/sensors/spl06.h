#pragma once

#include "stm32_i2c_driver.h"
#include <stdint.h>
#include <stdbool.h>

// I2C地址: 0X76
#define SPL06_I2C_HANDLE hi2c1
#define DETECTION_MAX_RETRY_COUNT   5

typedef struct {
	int16_t c0;
	int16_t c1;
	int32_t c00;
	int32_t c10;
	int16_t c01;
	int16_t c11;
	int16_t c20;
	int16_t c21;
	int16_t c30;
}spl06_coeffs_t;

#define SPL06_I2C_ADDR                         0x76
#define SPL06_DEFAULT_CHIP_ID                  0x10

#define SPL06_PRESSURE_START_REG               0x00
#define SPL06_PRESSURE_LEN                     3       // 24 bits, 3 bytes
#define SPL06_PRESSURE_B2_REG                  0x00    // Pressure MSB Register
#define SPL06_PRESSURE_B1_REG                  0x01    // Pressure middle byte Register
#define SPL06_PRESSURE_B0_REG                  0x02    // Pressure LSB Register
#define SPL06_TEMPERATURE_START_REG            0x03
#define SPL06_TEMPERATURE_LEN                  3       // 24 bits, 3 bytes
#define SPL06_TEMPERATURE_B2_REG               0x03    // Temperature MSB Register
#define SPL06_TEMPERATURE_B1_REG               0x04    // Temperature middle byte Register
#define SPL06_TEMPERATURE_B0_REG               0x05    // Temperature LSB Register
#define SPL06_PRESSURE_CFG_REG                 0x06    // Pressure config
#define SPL06_TEMPERATURE_CFG_REG              0x07    // Temperature config
#define SPL06_MODE_AND_STATUS_REG              0x08    // Mode and status
#define SPL06_INT_AND_FIFO_CFG_REG             0x09    // Interrupt and FIFO config
#define SPL06_INT_STATUS_REG                   0x0A    // Interrupt and FIFO config
#define SPL06_FIFO_STATUS_REG                  0x0B    // Interrupt and FIFO config
#define SPL06_RST_REG                          0x0C    // Softreset Register
#define SPL06_CHIP_ID_REG                      0x0D    // Chip ID Register
#define SPL06_CALIB_COEFFS_START               0x10
#define SPL06_CALIB_COEFFS_END                 0x21

#define SPL06_CALIB_COEFFS_LEN                 (SPL06_CALIB_COEFFS_END - SPL06_CALIB_COEFFS_START + 1)

// TEMPERATURE_CFG_REG
#define SPL06_TEMP_USE_EXT_SENSOR              (1<<7)

// MODE_AND_STATUS_REG
#define SPL06_MEAS_PRESSURE                    (1<<0)  // measure pressure
#define SPL06_MEAS_TEMPERATURE                 (1<<1)  // measure temperature

#define SPL06_MEAS_CFG_CONTINUOUS              (1<<2)
#define SPL06_MEAS_CFG_PRESSURE_RDY            (1<<4)
#define SPL06_MEAS_CFG_TEMPERATURE_RDY         (1<<5)
#define SPL06_MEAS_CFG_SENSOR_RDY              (1<<6)
#define SPL06_MEAS_CFG_COEFFS_RDY              (1<<7)

// INT_AND_FIFO_CFG_REG
#define SPL06_PRESSURE_RESULT_BIT_SHIFT        (1<<2)  // necessary for pressure oversampling > 8
#define SPL06_TEMPERATURE_RESULT_BIT_SHIFT     (1<<3)  // necessary for temperature oversampling > 8
#define SPL06_PRESSURE_OVERSAMPLING            8       // oversampling 8
#define SPL06_TEMPERATURE_OVERSAMPLING         8       // oversampling 8
#define SPL06_MEASUREMENT_TIME(oversampling)   ((2 + lrintf(oversampling * 1.6)) + 1) // ms

bool spl06_init(void);
int8_t spl06_samples_to_cfg_reg_value(uint8_t sample_rate);
int32_t spl06_raw_value_scale_factor(uint8_t oversampling_rate);
bool spl06_start_temperature_measurement(void);
bool spl06_read_temperature(void);
bool spl06_start_pressure_measurement(void);
bool spl06_read_pressure(void);
float spl06_compensate_temperature(int32_t temperature_raw);
float spl06_compensate_pressure(int32_t pressure_raw, int32_t temperature_raw);
bool spl06_calculate(float* pressure, float* temperature);
bool read_calibration_coefficients(void);
bool spl06_configure_measurements(void);
bool spl06Detect(void);
bool spl06_init(void);
void spl06_update(void);
