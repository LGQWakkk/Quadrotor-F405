#pragma once
// 20250703 Wakkk
#include <stdint.h>
#include <stdbool.h>
#include "system.h"
#include "i2c.h"
#include <math.h>

#define STM32_QMC5883L_I2C_HANDLE hi2c1

// QMC5883L数据结构
typedef struct{
    float mx; // Gauss
    float my; // Gauss
    float mz; // Gauss
    float temp;  // 芯片温度 degC
}qmc5883l_data_t;

// QMC5883L校准数据结构
// Model:
// (x - x_bias)/x_scale)**2 + ((y - y_bias)/y_scale)**2 + ((z - z_bias)/z_scale)**2 - 1
typedef struct{
    float x_scale;
    float y_scale;
    float z_scale;
    float x_bias;
    float y_bias;
    float z_bias;
    float average_scale; // 平均比例 用于恢复实际高斯值
}qmc5883l_calib_data_t;

// 对于STM32需要左移1bit
#define QMC5883L_ADDRESS (0x0D<<1)

#define REG_DATA_OUT_X_L 0x00
#define REG_DATA_OUT_X_H 0x01
#define REG_DATA_OUT_Y_L 0x02
#define REG_DATA_OUT_Y_H 0x03
#define REG_DATA_OUT_Z_L 0x04
#define REG_DATA_OUT_Z_H 0x05
#define REG_STATUS       0x06
#define REG_TEMP_DATA_L  0x07
#define REG_TEMP_DATA_H  0x08
#define REG_CONTROL1     0x09
#define REG_CONTROL2     0x0A
#define REG_PERIOD       0x0B
#define REG_CHIP_ID      0x0D
#define CHIP_ID          0xFF

// #define QMC5883L_REG_NUMS 13
#define QMC5883L_REG_NUMS  20

// 测量模式
#define QMC5883L_MODE_STANDBY       (0x00 << 0)
#define QMC5883L_MODE_CONTINUOUS    (0x01 << 0)
// 数据输出频率
#define QMC5883L_ODR_10HZ           (0x00 << 2)
#define QMC5883L_ODR_50HZ           (0x01 << 2)
#define QMC5883L_ODR_100HZ          (0x02 << 2)
#define QMC5883L_ODR_200HZ          (0x03 << 2)
// 量程
#define QMC5883L_RNG_2G             (0x00 << 4)
#define QMC5883L_RNG_8G             (0x01 << 4)
// 过采样比例(滤波系数)
#define QMC5883L_OSR_64             (0x03 << 6)
#define QMC5883L_OSR_128            (0x02 << 6)
#define QMC5883L_OSR_256            (0x01 << 6)
#define QMC5883L_OSR_512            (0x00 << 6)

// DOR: 0: normal, 1: data skipped for reading
#define QMC5883L_STATUS_DOR         (0x04)
// OVL: 0: normal, 1: data overflow
#define QMC5883L_STATUS_OVL         (0x02)
// DRDY: 0: no new data, 1: new data is ready
#define QMC5883L_STATUS_DRDY        (0x01)

#define QMC5883L_2G_SCALE           (12000.0f)   // LSB/Gauss
#define QMC5883L_8G_SCALE           (3000.0f)    // LSB/Gauss
#define QMC5883L_TMP_SCALE          (100.0f)     // LSB/Celsius

// Functions
uint8_t qmc5883l_read_byte(uint8_t reg);
void qmc5883l_read_buf(uint8_t reg, uint8_t *buf, uint8_t len);
void qmc5883l_write_byte(uint8_t reg,uint8_t data);
bool qmc5883l_probe(void);
bool qmc5883l_init(void);
bool qmc5883l_read(qmc5883l_data_t *data);
void qmc5883l_dump_regs(void);
float qmc5883l_get_mag_norm(qmc5883l_data_t *data);
float qmc5883l_apply_calib_data(qmc5883l_data_t *data, qmc5883l_calib_data_t *calib_data);
