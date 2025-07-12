#include "param.h"

// 调试功能数据包:
// AHRS参数设置

bool param_parse_frame(uint8_t *buf)
{
    if(!param_check_sum(buf, buf[31]))return false;
    switch(buf[0]){
        case FRAME_SO3_PARAM:
            return param_parse_so3_param(buf);
        default:
            return false;
    }
}

// 32Bytes数据包校验
// 前31字节数据包的校验和
bool param_check_sum(uint8_t *buf, uint8_t ref_sum)
{
    uint8_t sum = 0;
    for(uint8_t i=0; i<31; i++){
        sum += buf[i];
    }
    if(sum == ref_sum)return true;
    else return false;
}

// SO3控制器参数设置
// 共12个float参数 使用int16_t放大1000倍传输 实际设置范围最大为-32.768~32.767
bool param_parse_so3_param(uint8_t *buf)
{
    int16_t kr_1000[3];
    int16_t kw_1000[3];
    int16_t ki_1000[3];
    int16_t gain_1000[3];
    so3_controller_param_t _so3_param;
    memcpy(kr_1000, &buf[1], 6);
    memcpy(kw_1000, &buf[7], 6);
    memcpy(ki_1000, &buf[13], 6);
    memcpy(gain_1000, &buf[19], 6);
    _so3_param.kr[0] = (float)kr_1000[0]/1000.0f;
    _so3_param.kr[1] = (float)kr_1000[1]/1000.0f;
    _so3_param.kr[2] = (float)kr_1000[2]/1000.0f;
    _so3_param.kw[0] = (float)kw_1000[0]/1000.0f;
    _so3_param.kw[1] = (float)kw_1000[1]/1000.0f;
    _so3_param.kw[2] = (float)kw_1000[2]/1000.0f;
    _so3_param.ki[0] = (float)ki_1000[0]/1000.0f;
    _so3_param.ki[1] = (float)ki_1000[1]/1000.0f;
    _so3_param.ki[2] = (float)ki_1000[2]/1000.0f;
    _so3_param.gain[0] = (float)gain_1000[0]/1000.0f;
    _so3_param.gain[1] = (float)gain_1000[1]/1000.0f;
    _so3_param.gain[2] = (float)gain_1000[2]/1000.0f;
    so3_controller_set_param(&_so3_param);
    return true;
}
