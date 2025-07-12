import numpy as np
import matplotlib.pyplot as plt
imu_data_file = 'imu_data.npy'
imu_data = np.load(imu_data_file)
print(imu_data.shape)
# timestamp_ms | accx | accy | accz | gyrox | gyroy | gyroz
timestamp_ms = imu_data[:, 0]
acc_raw = imu_data[:, 1:4]
gyro_raw = imu_data[:, 4:7]

acc_g = acc_raw * 16.0 / 32768.0
gyro_dps = gyro_raw * 2000.0 / 32768.0

plt.subplot(2, 1, 1)
plt.plot(timestamp_ms, acc_g[:, 0], label='acc_x')
plt.plot(timestamp_ms, acc_g[:, 1], label='acc_y')
plt.plot(timestamp_ms, acc_g[:, 2], label='acc_z')
plt.xlabel('timestamp_ms')
plt.ylabel('acc_g')
plt.legend()

plt.subplot(2, 1, 2)
plt.plot(timestamp_ms, gyro_dps[:, 0], label='gyro_x')
plt.plot(timestamp_ms, gyro_dps[:, 1], label='gyro_y')
plt.plot(timestamp_ms, gyro_dps[:, 2], label='gyro_z')
plt.xlabel('timestamp_ms')
plt.ylabel('gyro_dps')
plt.legend()

plt.show()