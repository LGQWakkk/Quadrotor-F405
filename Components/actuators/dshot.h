#pragma once
// 20250703 DSHOT Driver For STM32F4 HAL LL
#include <stdint.h>
#include <stdbool.h>
#include "tim.h"
#include "system.h"

// DSHOT全部数值范围0-2047
// 0用于电调解锁
// 1-47用于遥测或者其他指令
// 1-5: 电调鸣叫 从低频到高频
// 6: ESC 版本信息或者序列号 通过遥测返回
// 7 8: 对应两个旋转方向
// 9 10： 3D模式开关 9:关 10:开
// 11: 获取ESC配置
// 12: 保存ESC配置
// 13: 遥测拓展信息展开 例如反馈温度、电压、电流等等
// 14: 拓展信息关闭
// 20 21: 也是切换旋转方向
// 22-29: 3个LED的亮灭控制
// 48-2047: 油门0-1999 油门有2000分辨率

// DSHOT SPEED
typedef enum {
    PWM_TYPE_DSHOT150=0,
    PWM_TYPE_DSHOT300,
    PWM_TYPE_DSHOT600,
} motorPwmProtocolTypes_e;

// 输出DSHOT使用的TIMER
#define DSHOT_TIMER_HANDLE htim3

// DSHOT 相关参数
#define DSHOT_BURST_LENGTH    18

#define DSHOT_TIMER_PERIOD    20     // 定时器计数周期(tick)
#define DSHOT_MOTOR_BIT_0     7
#define DSHOT_MOTOR_BIT_1     14

#define MOTOR_DSHOT600_HZ     12000000
#define MOTOR_DSHOT300_HZ     6000000
#define MOTOR_DSHOT150_HZ     3000000

uint16_t compose_dshot_packet(const uint16_t value, bool requestTelemetry);
void dshot_update_buffer(uint32_t *dmaBuffer, uint16_t packet);
void dshot_configure_dma_burst(void);
bool dshot_update_channel(uint8_t index, uint16_t value, bool requestTelemetry);
bool dshot_update_channel_norm(uint8_t index, float value, bool requestTelemetry);
bool dshot_disarm(uint8_t index);
void dshot_dma_cplt_callback(void);
void dshot_dma_error_callback(void);
void dshot_dma_burst_start(void);
void dshot_dma_burst_stop(void);
