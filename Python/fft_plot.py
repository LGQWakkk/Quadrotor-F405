# 原始信号FFT频谱分析
# 20250710 Wakkk
import numpy as np
import matplotlib.pyplot as plt
imu_data_file = 'imu_data.npy'
imu_data = np.load(imu_data_file)
# timestamp_ms | accx | accy | accz | gyrox | gyroy | gyroz | throttle
timestamp = imu_data[:, 0]
throttle = imu_data[:, 7]
acc = imu_data[:, 1:4]
gyro = imu_data[:, 4:7]
# 采样频率
fs = 1000.0 # Hz
data_len = len(timestamp)

# FFT分析
def plot_signal_fft(signal, fs):
    n = len(signal)
    fft_result = np.fft.fft(signal)
    magnitude = np.abs(fft_result) / n
    freq = np.fft.fftfreq(n, 1/fs)
    # 取正频率
    half_n = n // 2
    freq_pos = freq[:half_n]
    magnitude_pos = 2 * magnitude[:half_n]  # 单边谱需要乘以2
    plt.figure(figsize=(12, 6))
    plt.subplot(2, 1, 1)
    time = np.arange(n) / fs
    plt.plot(time, signal)
    plt.title('time domain signal')
    plt.xlabel('time')
    plt.ylabel('amplitude')
    plt.grid(True)
    
    plt.subplot(2, 1, 2)
    plt.plot(freq_pos, magnitude_pos)
    plt.title('frequency domain signal')
    plt.xlabel('frequency (Hz)')
    plt.ylabel('amplitude')
    plt.grid(True)
    plt.tight_layout()
    plt.show()

plot_signal_fft(acc[:, 1], fs)
# plot_signal_fft(gyro[:, 0], fs)
