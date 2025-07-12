# 20250710 Wakkk
import serial
from serial.tools import list_ports
import struct
import numpy as np

def find_serial_port():
    available_ports = list_ports.comports()
    if not available_ports:
        print("No available serial ports found")
        return None
    first_port = available_ports[0]
    print(f"Using Serial Port: {first_port.device}")
    try:
        ser = serial.Serial(first_port.device, baudrate=1000000, timeout=1)
        print("Serial Port Opened")
        return ser
    except serial.SerialException as e:
        print(f"Error Opening Serial Port: {e}")
        return None

imu_data = []
def parse_imu_data(data):
    global imu_data
    # if data[0] != 0xAA or data[17] != 0x55:
    if data[0] != 0xAA:
        return False
    timestamp_ms = struct.unpack('<L', data[1:5])[0]
    gyro = struct.unpack('<fff', data[5:17])
    acc = struct.unpack('<fff', data[17:29])
    throttle = struct.unpack('<f', data[29:33])[0]

    imu_data.append([timestamp_ms, acc[0], acc[1], acc[2], gyro[0], gyro[1], gyro[2], throttle])

    # print(f"throttle: {throttle}")
    # print(f"acc: {acc[0]} {acc[1]} {acc[2]}")
    # print(f"gyro: {gyro[0]} {gyro[1]} {gyro[2]}")

    # acc_ori = np.array([acc_raw[0], -acc_raw[1], acc_raw[2]])
    # gyro_ori = np.array([-gyro_raw[0], gyro_raw[1], gyro_raw[2]])
    # print(f"acc: {acc_ori[0]} {acc_ori[1]} {acc_ori[2]}")
    # print(f"gyro: {gyro_ori[0]} {gyro_ori[1]} {gyro_ori[2]}")
    # imu_data.append([timestamp_ms, acc_raw[0], acc_raw[1], acc_raw[2], gyro_raw[0], gyro_raw[1], gyro_raw[2]])
    return True

serial_port = find_serial_port()
if serial_port:
    data_packet_size = 33
    while True:
        data = serial_port.read(data_packet_size)
        if len(data) == data_packet_size:
            if parse_imu_data(data):
                pass
            else:
                print("Invalid Data Received")
                serial_port.reset_input_buffer()
        else:
            print("Invalid Data Received")
            serial_port.reset_input_buffer()
        
        # Check Data
        data_len = len(imu_data)
        if data_len > 8000:
            print("Data List Length:", data_len)
            data_list_np = np.array(imu_data)
            np.save("imu_data.npy", data_list_np)
            break 
    print("Data Saved")
    serial_port.close()
