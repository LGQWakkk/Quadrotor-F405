import numpy as np
import matplotlib.pyplot as plt
imu_data_file = 'imu_data.npy'
imu_data = np.load(imu_data_file)

# timestamp_ms | accx | accy | accz | gyrox | gyroy | gyroz | throttle
timestamp = imu_data[:, 0]
throttle = imu_data[:, 7]
acc = imu_data[:, 1:4]
gyro = imu_data[:, 4:7]

# 绘制油门和时间图表
# plt.plot(timestamp, throttle)
# plt.xlabel('Time (ms)')
# plt.ylabel('Throttle')
# plt.show()

# 绘制加速度和时间图表
plt.plot(timestamp, acc)
plt.xlabel('Time (ms)')
plt.ylabel('Acceleration (m/s^2)')
plt.show()
