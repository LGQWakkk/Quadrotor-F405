// 20250703 Wakkk
#include "qmc5883l.h"

// 寄存器读取一个字节
uint8_t qmc5883l_read_byte(uint8_t reg)
{
	uint8_t data;
	HAL_I2C_Mem_Read(&STM32_QMC5883L_I2C_HANDLE, QMC5883L_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, 1000);
	return data;
}

// 寄存器读取多个字节
void qmc5883l_read_buf(uint8_t reg, uint8_t *buf, uint8_t len)
{
    HAL_I2C_Mem_Read(&STM32_QMC5883L_I2C_HANDLE, QMC5883L_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, buf, len, 1000);
}

// 寄存器写入一个字节
void qmc5883l_write_byte(uint8_t reg, uint8_t data)
{
    HAL_I2C_Mem_Write(&STM32_QMC5883L_I2C_HANDLE, QMC5883L_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, 1000);
}

// 检测QMC5883L是否位于总线上
bool qmc5883l_probe(void)
{
    uint8_t id = qmc5883l_read_byte(REG_CHIP_ID);
    if(id == CHIP_ID){
        return true;
    }else{
        return false;
    }
}

// 初始化QMC5883L
bool qmc5883l_init(void)
{
    bool ret = qmc5883l_probe();
    if(!ret){
        slog("Error: QMC5883L Not Found.\n");
        return false;
    }
    qmc5883l_write_byte(REG_CONTROL2, 0x80);// 软件复位
    HAL_Delay(2);
    qmc5883l_write_byte(REG_PERIOD, 0x01);// 设置为连续测量模式
    HAL_Delay(2);
    // 20250622 发现这个QMC5883L_ODR_100HZ设置好像不太好使 量程是好使的
    uint8_t config = QMC5883L_MODE_CONTINUOUS | QMC5883L_ODR_100HZ | QMC5883L_RNG_8G | QMC5883L_OSR_512;
    qmc5883l_write_byte(REG_CONTROL1, config); // 配置
    return true;
}

bool qmc5883l_read(qmc5883l_data_t *data)
{
    // 首先判断DRDY
    uint8_t status = qmc5883l_read_byte(REG_STATUS);
    if(status & QMC5883L_STATUS_DRDY){
        // 读取数据
        int16_t raw[3]; // 原始数据 int16_t
        uint8_t buf[6+3+1]; // 原始buffer(MAG TEMP) 这里因为地址0固定0xFF的问题多读取一个字节
        qmc5883l_read_buf(REG_DATA_OUT_X_L, buf, 6+3+1); // 读取磁场数据
        // 处理原始数据
        raw[0] = (int16_t)(((uint16_t)buf[1+1] << 8) | buf[0+1]);
        raw[1] = (int16_t)(((uint16_t)buf[3+1] << 8) | buf[2+1]);
        raw[2] = (int16_t)(((uint16_t)buf[5+1] << 8) | buf[4+1]);
        // slog("Mag Raw Data: %d \t %d \t %d\n", raw[0], raw[1], raw[2]);
        // slog("Mag Buf Data: ");
        // for(uint8_t i=0; i<6+3+1; i++){
        //     slog(" 0x%02X", buf[i]);
        // }
        // slog("\n");

        // 转换为Gauss
        data->mx = (float)raw[0] / QMC5883L_8G_SCALE;
        data->my = (float)raw[1] / QMC5883L_8G_SCALE;
        data->mz = (float)raw[2] / QMC5883L_8G_SCALE;
        int16_t temp = (int16_t)(((uint16_t)buf[8+1] << 8) | buf[7+1]);
        data->temp = (float)temp / QMC5883L_TMP_SCALE;

        return true;
    }else{
        return false;// 无新数据
    }
}

// 获取磁力模值(Gauss)
float qmc5883l_get_mag_norm(qmc5883l_data_t *data)
{
    float norm = sqrt(data->mx*data->mx + data->my*data->my + data->mz*data->mz);
    return norm;
}

// 对磁力计施加校准数据 返回值为校准之后的模值(Gauss)
float qmc5883l_apply_calib_data(qmc5883l_data_t *data, qmc5883l_calib_data_t *calib_data)
{
    float mx_corr = (data->mx - calib_data->x_bias) / calib_data->x_scale;
    float my_corr = (data->my - calib_data->y_bias) / calib_data->y_scale;
    float mz_corr = (data->mz - calib_data->z_bias) / calib_data->z_scale;
    float norm_corr = calib_data->average_scale * sqrt(mx_corr*mx_corr + my_corr*my_corr + mz_corr*mz_corr);
    // 更新校准之后的数值 Gauss
    data->mx = mx_corr * calib_data->average_scale;
    data->my = my_corr * calib_data->average_scale;
    data->mz = mz_corr * calib_data->average_scale;
    return norm_corr;
}

// 实际情况:
// 从0x00开始读取20个字节:
// Data Regs: 0xFF [0x45 0x04 0x71 0xFD 0x78 0x00] 0x05 [0x9B 0xFC] [0x19] 0x00 0x01 0x01 0x00 0x00 0x00 0x00 0x00 0x00
// 所以这个第一个寄存器应该就是最后一个0xFF 本应该是最后一个寄存器 不知道为什么出现在了第一个
void qmc5883l_dump_regs(void)
{
    uint8_t buffer[QMC5883L_REG_NUMS];
    qmc5883l_read_buf(0x00, buffer, QMC5883L_REG_NUMS);
    slog("Data Regs:");
    for(int i=0; i<QMC5883L_REG_NUMS; i++){
        slog(" 0x%02X", buffer[i]);
    }
    slog("\n");
}
