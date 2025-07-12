#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "main.h"
#include "i2c.h"

typedef struct{
    uint8_t addr;
    I2C_HandleTypeDef *handle;
}stm32_i2c_driver_t;

bool stm32_i2c_write(stm32_i2c_driver_t* driver, uint8_t reg, uint8_t* data, uint16_t len);
bool stm32_i2c_read(stm32_i2c_driver_t* driver, uint8_t reg, uint8_t* data, uint16_t len);
bool stm32_i2c_write_byte(stm32_i2c_driver_t* driver, uint8_t reg, uint8_t data);
uint8_t stm32_i2c_read_byte(stm32_i2c_driver_t* driver, uint8_t reg);
