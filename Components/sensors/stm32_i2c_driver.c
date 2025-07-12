#include "stm32_i2c_driver.h"

// 向寄存器写入多个字节
bool stm32_i2c_write(stm32_i2c_driver_t* driver, uint8_t reg, uint8_t* data, uint16_t len)
{
    HAL_I2C_Mem_Write(driver->handle, driver->addr, reg, I2C_MEMADD_SIZE_8BIT, data, len, 0xFFFF);
    return true;
}

// 从寄存器读取多个字节
bool stm32_i2c_read(stm32_i2c_driver_t* driver, uint8_t reg, uint8_t* data, uint16_t len)
{
    HAL_I2C_Mem_Read(driver->handle, driver->addr, reg, I2C_MEMADD_SIZE_8BIT, data, len, 0xFFFF);
    return true;
}

// 向寄存器写入一个字节
bool stm32_i2c_write_byte(stm32_i2c_driver_t* driver, uint8_t reg, uint8_t data)
{
    HAL_I2C_Mem_Write(driver->handle, driver->addr, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, 0xFFFF);
    return true;
}

// 从寄存器读取一个字节 直接返回读取到的数据
uint8_t stm32_i2c_read_byte(stm32_i2c_driver_t* driver, uint8_t reg)
{
    uint8_t data;
    HAL_I2C_Mem_Read(driver->handle, driver->addr, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, 0xFFFF);
    return data;
}
