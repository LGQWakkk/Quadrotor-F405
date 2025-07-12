// 20250703 DSHOT Driver For STM32F4 HAL LL
#include "dshot.h"

// 关于TIM配置
// MCU主频168MHz
// 定时器分频值 14-1
// 定时器自动重装载值 20-1
// 使用TIM1_UP触发DMA传输 传输方向为内存到外设 传输大小为Word
// 使用TIM1 CH1-CH4 输出DSHOT信号
// DMA2 STREAM5

// 4个通道 DMA BURST BUFFER
uint32_t dshot_dma_buffer[DSHOT_BURST_LENGTH * 4]; //18x4 = 72 words

// 构建DSHOT数据包 内部函数
uint16_t compose_dshot_packet(const uint16_t value, bool requestTelemetry)
{
    uint16_t packet = (value << 1) | (requestTelemetry ? 1 : 0);
    // compute checksum
    int csum = 0;
    int csum_data = packet;
    for (int i = 0; i < 3; i++) {
        csum ^=  csum_data;   // xor data by nibbles
        csum_data >>= 4;
    }
    csum &= 0xf;
    packet = (packet << 4) | csum;// append checksum
    return packet;
}

// 填充DMA BURST BUFFER 内部函数
void dshot_update_buffer(uint32_t *dmaBuffer, uint16_t packet)
{
    int i;
    for (i = 0; i < 16; i++) {// i: bit index
        dmaBuffer[i * 4] = (packet & 0x8000) ? DSHOT_MOTOR_BIT_1 : DSHOT_MOTOR_BIT_0;  // MSB first
        packet <<= 1;
    }
    dmaBuffer[i++ * 4] = 0;// 填充最后两个bit对应的buffer
    dmaBuffer[i++ * 4] = 0;// 填充最后两个bit对应的buffer
}

// 配置DMA BURST传输 要求TIM1已经启动
void dshot_configure_dma_burst(void)
{    
    // 配置定时器端BURST基地址以及自增距离
    LL_TIM_ConfigDMABurst(TIM1, LL_TIM_DMABURST_BASEADDR_CCR1, LL_TIM_DMABURST_LENGTH_4TRANSFERS);
    LL_DMA_ConfigAddresses(DMA2, LL_DMA_STREAM_5, (uint32_t)dshot_dma_buffer, (uint32_t)&TIM1->DMAR, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
    LL_DMA_EnableIT_TC(DMA2, LL_DMA_STREAM_5);     // 传输完毕中断
    LL_DMA_EnableIT_TE(DMA2, LL_DMA_STREAM_5);     // 传输错误中断

    LL_TIM_ClearFlag_UPDATE(TIM1);
    LL_TIM_EnableDMAReq_UPDATE(TIM1);                   // 使能TIM1更新 DMA请求
    LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1);  // 使能通道
    LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH2);  // 使能通道
    LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH3);  // 使能通道
    LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH4);  // 使能通道
    LL_TIM_EnableAllOutputs(TIM1); // 20250703 使能所有输出 必须添加
    LL_TIM_EnableCounter(TIM1);                         // 使能计数器
    // LL_TIM_GenerateEvent_UPDATE(TIM1);               // 强制更新
}

void dshot_dma_burst_start(void)
{
    LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_5, DSHOT_BURST_LENGTH*4); // 设置DMA传输长度
    LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_5); // 启动DMA传输
}

void dshot_dma_burst_stop(void)
{
    LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_5);
}

// 更新单个DSHOT通道的输出数值 不会立即发送
// index: 电机索引 0-3
// value: 通道数值 11bit 其中油门数值48-2047
// requestTelemetry: 是否请求遥测数据
bool dshot_update_channel(uint8_t index, uint16_t value, bool requestTelemetry)
{
    if(index >= 4)return false;
    uint16_t packet = compose_dshot_packet(value, requestTelemetry);      // 构建DSHOT数据包
    dshot_update_buffer(&(dshot_dma_buffer[index]), packet);              // 填充通道buffer
    return true;
}

// 归一化输入 value: 0.0-1.0
bool dshot_update_channel_norm(uint8_t index, float value, bool requestTelemetry)
{
    if(index >= 4)return false;
    if(value < 0.0f){value = 0.0f;}
    if(value > 1.0f){value = 1.0f;}
    uint16_t throttle_us = (uint16_t)(value * (2000.0f - 60.0f));
    throttle_us += 48 + 60; // 加上油门最小值
    if(throttle_us > 2047)throttle_us = 2047;
    return dshot_update_channel(index, throttle_us, requestTelemetry);
}

// DISARM
bool dshot_disarm(uint8_t index)
{
    if(index >= 4)return false;
    return dshot_update_channel(index, 0, false);
}

// DSHOT BURST DMA 传输完成回调函数
void dshot_dma_cplt_callback(void)
{
    dshot_dma_burst_stop();
}

// DSHOT BURST DMA 传输错误回调函数
void dshot_dma_error_callback(void)
{

}
