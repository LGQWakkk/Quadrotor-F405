import serial
import time
import struct
import numpy as np

ser = serial.Serial('COM35', 115200)
if ser is None:
    print("Cannot open serial port")
    exit()

# 帧格式: 帧头0xAA 随后31字节数据 最后一字节为和校验
# 帧头: 0xAA
# 数据: 31字节
# 校验: 1字节

# bool param_parse_so3_param(uint8_t *buf)
# {
#     int16_t kr_1000[3];
#     int16_t kw_1000[3];
#     int16_t ki_1000[3];
#     int16_t gain_1000[3];
#     so3_controller_param_t _so3_param;
#     memcpy(kr_1000, &buf[1], 6);
#     memcpy(kw_1000, &buf[7], 6);
#     memcpy(ki_1000, &buf[13], 6);
#     memcpy(gain_1000, &buf[19], 6);
#     _so3_param.kr[0] = (float)kr_1000[0]/1000.0f;
#     _so3_param.kr[1] = (float)kr_1000[1]/1000.0f;
#     _so3_param.kr[2] = (float)kr_1000[2]/1000.0f;
#     _so3_param.kw[0] = (float)kw_1000[0]/1000.0f;
#     _so3_param.kw[1] = (float)kw_1000[1]/1000.0f;
#     _so3_param.kw[2] = (float)kw_1000[2]/1000.0f;
#     _so3_param.ki[0] = (float)ki_1000[0]/1000.0f;
#     _so3_param.ki[1] = (float)ki_1000[1]/1000.0f;
#     _so3_param.ki[2] = (float)ki_1000[2]/1000.0f;
#     _so3_param.gain[0] = (float)gain_1000[0]/1000.0f;
#     _so3_param.gain[1] = (float)gain_1000[1]/1000.0f;
#     _so3_param.gain[2] = (float)gain_1000[2]/1000.0f;
#     so3_controller_set_param(&_so3_param);
#     return true;
# }

# 发送SO3参数
def pack_so3_param_data(kr, kw, ki, gain):
    data = bytearray(33)
    data[0] = 0xAA
    data[1] = 0x20 # SO3参数帧头
    struct.pack_into('h', data, 2, int(kr[0]*1000))
    struct.pack_into('h', data, 4, int(kr[1]*1000))
    struct.pack_into('h', data, 6, int(kr[2]*1000))
    struct.pack_into('h', data, 8, int(kw[0]*1000))
    struct.pack_into('h', data, 10, int(kw[1]*1000))
    struct.pack_into('h', data, 12, int(kw[2]*1000))
    struct.pack_into('h', data, 14, int(ki[0]*1000))
    struct.pack_into('h', data, 16, int(ki[1]*1000))
    struct.pack_into('h', data, 18, int(ki[2]*1000))
    struct.pack_into('h', data, 20, int(gain[0]*1000))
    struct.pack_into('h', data, 22, int(gain[1]*1000))
    struct.pack_into('h', data, 24, int(gain[2]*1000))
    # 计算校验和
    checksum = 0
    # 从1开始到32
    for i in range(1, 32):
        checksum += data[i]
    data[32] = checksum & 0xFF
    return data

# 发送到串口
def send_so3_param(kr, kw, ki, gain):
    data = pack_so3_param_data(kr, kw, ki, gain)
    ser.write(data)

print("Sending SO3 parameters...")
kr = np.array([1.3, 1.3, 0.5])
kw = np.array([0.5, 0.5, 0.4])
ki = np.array([0.0, 0.0, 0.0])
gain = np.array([-1.0, -1.0, -1.0])
send_so3_param(kr, kw, ki, gain)
print("SO3 parameters sent.")
