#pragma once
// 20250630 Wakkk For HAL LL
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "system.h"

#define PACKED __attribute__((__packed__))

#define CRSF_MAX_CHANNELS   24U         // Maximum number of channels from crsf datastream
#define CRSF_FRAMELEN_MAX   64U         // maximum possible framelength
#define CRSF_HEADER_LEN     2U          // header length
#define CRSF_FRAME_PAYLOAD_MAX (CRSF_FRAMELEN_MAX - CRSF_HEADER_LEN)     // maximum size of the frame length field in a packet
#define CRSF_FRAME_LENGTH_MIN 2         // min value for _frame.length
#define ELRS_BAUDRATE      420000U
#define CRSF_HEADER_TYPE_LEN     (CRSF_HEADER_LEN + 1)           // header length including type
#define CRSF_FRAMETYPE_RC_CHANNELS_PACKED 0x16  // RC 通道数据帧
#define CRSF_ADDRESS_FLIGHT_CONTROLLER 0xC8 

////////////////////////////////////////////////////////////////////////////////////
// 对于RadioMaster POCKET遥控器 最大有效通道数量是9
// 模拟通道索引
#define ROLL_INDEX      0
#define PITCH_INDEX     1
#define THROTTLE_INDEX  2   
#define YAW_INDEX       3
// 定义摇杆模拟通道数值范围 根据遥控器实际情况调整
#define ROLL_MIN 988
#define ROLL_MID 1500
#define ROLL_MAX 2011
#define PITCH_MIN 988
#define PITCH_MID 1500
#define PITCH_MAX 2011
#define THROTTLE_MIN 988
#define THROTTLE_MID 1500
#define THROTTLE_MAX 2011
#define YAW_MIN 988
#define YAW_MID 1500
#define YAW_MAX 2011
// 二段开关识别电平
#define BUTTON_THRESHOLD      1500
// 三段开关识别电平
#define SWITCH3_UP_THRESHOLD    1750
#define SWITCH3_DOWN_THRESHOLD  1250
// CH4 CH7 为自锁开关
// CH8 为回弹开关
// CH5 CH6 为三段开关
#define BUTTON1_INDEX   4   // 自锁开关
#define BUTTON2_INDEX   7   // 自锁开关
#define BUTTON3_INDEX   8   // 回弹开关
#define SWITCH1_INDEX   5   // 三段开关
#define SWITCH2_INDEX   6   // 三段开关
#define SWITCH3_NUM     2   // 三段开关数量
#define BUTTON_NUM      3   // 按键数量
#define ANALOG_NUM      4   // 模拟输入通道数量 这里认为是只含有roll pitch throttle yaw

// 三段开关
typedef enum{
    SWITCH3_UP = 0,
    SWITCH3_MID = 1,
    SWITCH3_DOWN = 2,
    SWITCH_INVALID = 3  // 按钮不存在或者无效
} Switch3;

// RadioMaster POCKET
typedef struct{
    float roll;
    float pitch;
    float throttle;
    float yaw;
    bool button1;
    bool button2;
    bool button3;
    Switch3 switch1;
    Switch3 switch2;
    uint32_t timestamp_us;  //接收时间戳
} rc_data_t;
////////////////////////////////////////////////////////////////////////////////////

// CRSF帧数据格式
typedef struct{
    uint8_t device_address;
    uint8_t length;
    uint8_t type;
    uint8_t payload[CRSF_FRAME_PAYLOAD_MAX - 1]; // type is already accounted for
}PACKED CRSF_Frame;

// 11bit 8通道数据结构定义
typedef struct{
#if __BYTE_ORDER != __LITTLE_ENDIAN
#error "Only supported on little-endian architectures"
#endif
    uint32_t ch0 : 11;
    uint32_t ch1 : 11;
    uint32_t ch2 : 11;
    uint32_t ch3 : 11;
    uint32_t ch4 : 11;
    uint32_t ch5 : 11;
    uint32_t ch6 : 11;
    uint32_t ch7 : 11;
}PACKED Channels11Bit_8Chan_t;

void crsf_reset_buffer(void);
void crsf_process_byte(uint8_t byte);
void crsf_check_frame(void);
bool crsf_decode_crsf_packet(void);
void decode_11bit_channels(const uint8_t* buffer, uint8_t nchannels, uint16_t *values, uint16_t mult, uint16_t div, uint16_t offset);
uint8_t crc8_dvb_s2_update(uint8_t crc, const void *data, uint32_t length);
uint8_t crc8_dvb_s2(uint8_t crc, uint8_t a);
uint8_t crc8_dvb(uint8_t crc, uint8_t a, uint8_t seed);

extern uint8_t _frame_ofs;
extern CRSF_Frame _frame;
extern uint16_t output_channels[CRSF_MAX_CHANNELS];
extern rc_data_t crsf_data; // 最新RC数据

Switch3 crsf_switch3(uint16_t value);
Switch3 crsf_get_switch3(uint8_t index);
bool crsf_button(uint16_t value);
bool crsf_get_button(uint8_t index);
float crsf_bidir_norm(uint16_t value, uint16_t min, uint16_t mid, uint16_t max);
float crsf_norm(uint16_t value, uint16_t min, uint16_t max);
float crsf_get_channel_norm(uint8_t index);
void crsf_update_data(void);
