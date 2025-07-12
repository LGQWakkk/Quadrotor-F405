// 20250630 Wakkk For HAL LL
#include "crsf.h"
#include "main.h"

uint8_t _frame_ofs;                         // 帧内部索引(指针)
CRSF_Frame _frame;                          // 全局缓冲帧buffer 
uint8_t *_frame_bytes = (uint8_t*)&_frame;  // 帧缓冲区指针
uint16_t output_channels[CRSF_MAX_CHANNELS];// 输出通道数据
rc_data_t crsf_data; // 最新RC数据

#define CRSF_UART_TIMEOUT_MS 200            // 接收超时时间(ms) 超过此时间未更新视为断开连接

// 用于电机解锁的通道索引
#define ARM_CHANNEL_INDEX 4
#define ARM_CHANNEL_THRESHOLD 1500 // 解锁阈值

// 复位接收缓冲区
void crsf_reset_buffer(void)
{
    for(uint8_t i = 0; i < sizeof(_frame); i++){
        _frame_bytes[i] = 0;
    }
    _frame_ofs = 0;// 复位指针
}

// 处理新收到的一个字节
void crsf_process_byte(uint8_t byte)
{
    if (_frame_ofs >= sizeof(_frame)){crsf_reset_buffer();}// 防止缓冲区溢出
    _frame_bytes[_frame_ofs++] = byte;//保存单字节数据并自增指针
    crsf_check_frame();// 判断帧有效性
}

// 判断帧是否有效
void crsf_check_frame(void)
{
    if(_frame_ofs >= sizeof(_frame)){crsf_reset_buffer();return;}// 防止溢出
    if(_frame.device_address != CRSF_ADDRESS_FLIGHT_CONTROLLER){crsf_reset_buffer();return;}// 必须要求数据包帧头是0xC8
    if(_frame_ofs >= CRSF_HEADER_TYPE_LEN && _frame.length > CRSF_FRAME_PAYLOAD_MAX){crsf_reset_buffer();return;}// LEN过大 数据包错误
    if(_frame_ofs >= _frame.length + CRSF_HEADER_LEN){// 帧接收完整 计算CRC校验
        const uint8_t crc = crc8_dvb_s2_update(0, &_frame_bytes[CRSF_HEADER_LEN], _frame.length - 1);
        if (crc != _frame.payload[_frame.length - 2]){crsf_reset_buffer();return;}  //CRC校验失败
        crsf_decode_crsf_packet();  // 帧接收完毕 进行解码
        crsf_reset_buffer();        // 解析完毕之后清空缓冲区
        crsf_update_data();         // 更新解析数据
        return;
    }
}

// 收到一个完整的CRSF数据帧之后进行解析
bool crsf_decode_crsf_packet(void)
{
    switch (_frame.type) {//根据帧类型调用不同的解析函数
        case CRSF_FRAMETYPE_RC_CHANNELS_PACKED://RC通道数据 0x16
            decode_11bit_channels((const uint8_t*)(&_frame.payload), CRSF_MAX_CHANNELS, output_channels, 5U, 8U, 880U);
            break;
        default:
            break;
    }
    return true;
}

// buffer: 要解析的缓冲区指针
// nchannels: 要解析的通道数量 一般为16
// values: 解析后的数据存放数组
// mult: 通道数值倍数
// div: 通道数值除数
// offset: 通道数值偏移量
void decode_11bit_channels(const uint8_t* buffer, uint8_t nchannels, uint16_t *values, uint16_t mult, uint16_t div, uint16_t offset)
{
    while (nchannels >= 8) {
        Channels11Bit_8Chan_t* channels = (Channels11Bit_8Chan_t*) buffer;//将buffer转换为8通道数据包格式
        values[0] = (((int32_t)(channels->ch0) * mult) / div + offset);
        values[1] = (((int32_t)(channels->ch1) * mult) / div + offset);
        values[2] = (((int32_t)(channels->ch2) * mult) / div + offset);
        values[3] = (((int32_t)(channels->ch3) * mult) / div + offset);
        values[4] = (((int32_t)(channels->ch4) * mult) / div + offset);
        values[5] = (((int32_t)(channels->ch5) * mult) / div + offset);
        values[6] = (((int32_t)(channels->ch6) * mult) / div + offset);
        values[7] = (((int32_t)(channels->ch7) * mult) / div + offset);
        nchannels -= 8;
        buffer += sizeof(*channels);//移动buffer指针到下8个通道
        values += 8;
    }
}

// crc8 from betaflight
uint8_t crc8_dvb_s2_update(uint8_t crc, const void *data, uint32_t length)
{
    const uint8_t *p = (const uint8_t *)data;
    const uint8_t *pend = p + length;
    for (; p != pend; p++) {
        crc = crc8_dvb_s2(crc, *p);
    }
    return crc;
}

// crc8 from betaflight
uint8_t crc8_dvb_s2(uint8_t crc, uint8_t a)
{
    return crc8_dvb(crc, a, 0xD5);
}

// crc8 from betaflight
uint8_t crc8_dvb(uint8_t crc, uint8_t a, uint8_t seed)
{
    crc ^= a;
    for (uint8_t i = 0; i < 8; ++i) {
        if (crc & 0x80) {
            crc = (crc << 1) ^ seed;
        } else {
            crc = crc << 1;
        }
    }
    return crc;
}
////////////////////////////////////////////////////////////////////////////////////
// 判断三段SWITCH电平
Switch3 crsf_switch3(uint16_t value)
{
    if(value > SWITCH3_UP_THRESHOLD){
        return SWITCH3_UP;
    }else if(value < SWITCH3_DOWN_THRESHOLD){
        return SWITCH3_DOWN;
    }else{
        return SWITCH3_MID;
    }
}

// 获取三段开关位置
// index: 三段开关索引 0-1
Switch3 crsf_get_switch3(uint8_t index)
{
    if(index < SWITCH3_NUM){
        if(index == 0){//SWITCH1_INDEX
            uint16_t value = output_channels[SWITCH1_INDEX];
            return crsf_switch3(value);
        }else if(index == 1){//SWITCH2_INDEX
            uint16_t value = output_channels[SWITCH2_INDEX];
            return crsf_switch3(value);
        }
    }else{
        return SWITCH_INVALID;// 无效索引
    }
}

// 根据开关电平判断状态
bool crsf_button(uint16_t value)
{
    if(value > BUTTON_THRESHOLD){
        return true;
    }else{
        return false;
    }
}

// 获取开关电平
// 开关置为1返回true 开关无效或者或者没有按下返回false
bool crsf_get_button(uint8_t index)
{
    if(index < BUTTON_NUM){
        if(index ==0){
            uint16_t value = output_channels[BUTTON1_INDEX];
            return crsf_button(value);
        }else if(index ==1){
            uint16_t value = output_channels[BUTTON2_INDEX];
            return crsf_button(value);
        }else if(index ==2){
            uint16_t value = output_channels[BUTTON3_INDEX];
            return crsf_button(value);
        }else{
            return false;
        }
    }else{
        return false;
    }
}

// 双向NORM 含有通道限幅
float crsf_bidir_norm(uint16_t value, uint16_t min, uint16_t mid, uint16_t max)
{
    if(value < min)value = min;
    if(value > max)value = max;
    if(value < mid){
        return (float)(value - mid) / (float)(mid - min); // <0
    }else{
        return (float)(value - mid) / (float)(max - mid); // >0
    }
}

// 单向NORM 含有通道限幅
float crsf_norm(uint16_t value, uint16_t min, uint16_t max)
{
    if(value < min)value = min;
    if(value > max)value = max;
    return (float)(value - min) / (float)(max - min);
}

// 获取模拟通道归一化输入
// 对于ROLLL PITCH YAW 范围为-1~1
// 对于THROTTLE 范围为0-1
// 无效通道返回0
float crsf_get_channel_norm(uint8_t index)
{
    uint16_t value = 0;
    switch(index){
        case ROLL_INDEX:    
            value = output_channels[ROLL_INDEX];
            return crsf_bidir_norm(value, ROLL_MIN, ROLL_MID, ROLL_MAX);
        case PITCH_INDEX:
            value = output_channels[PITCH_INDEX];
            return crsf_bidir_norm(value, PITCH_MIN, PITCH_MID, PITCH_MAX);
        case YAW_INDEX:
            value = output_channels[YAW_INDEX];
            return crsf_bidir_norm(value, YAW_MIN, YAW_MID, YAW_MAX);
        case THROTTLE_INDEX:
            value = output_channels[THROTTLE_INDEX];
            return crsf_norm(value, THROTTLE_MIN, THROTTLE_MAX);
        default:
            return 0.0f;
    }
}

// 更新最新RC数据
void crsf_update_data(void)
{
    crsf_data.timestamp_us   = get_system_1khz_tick();
    crsf_data.roll           = crsf_get_channel_norm(ROLL_INDEX);
    crsf_data.pitch          = -crsf_get_channel_norm(PITCH_INDEX); // 20250704 PITCH需反向
    crsf_data.throttle       = crsf_get_channel_norm(THROTTLE_INDEX);
    crsf_data.yaw            = crsf_get_channel_norm(YAW_INDEX);
    crsf_data.button1        = crsf_get_button(0);
    crsf_data.button2        = crsf_get_button(1);
    crsf_data.button3        = crsf_get_button(2);
    crsf_data.switch1        = crsf_get_switch3(0);
    crsf_data.switch2        = crsf_get_switch3(1);
}
