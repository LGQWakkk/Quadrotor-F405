# ButterWorth 滤波器参数设计 和Scipy进行互补验证
# 三阶滤波器
# 使用scipy设计参数
from scipy.signal import butter, lfilter
import numpy as np
import matplotlib.pyplot as plt

# https://docs.scipy.org/doc/scipy/reference/generated/scipy.signal.butter.html

# Numerator (b分子 前向) and denominator (a分母 反馈) polynomials of the IIR filter
def butter3_filter_design(cutoff, fs, order=3):
    nyq = 0.5 * fs  # 计算奈奎斯特采样频率
    normal_cutoff = cutoff / nyq  # 计算归一化截止频率
    # 设计一个低通Butterworth滤波器，返回b和a系数
    b, a = butter(order, normal_cutoff, btype='lowpass', analog=False)
    # return Butter3(b, a)
    return b, a   # 返回系数 前向 反馈

class Butter3:
    def __init__(self, b, a):
        # 初始化前向通路系数和反馈通路系数
        self.B = b
        self.A = a
        # 初始化输入和输出缓冲区
        self.X = np.zeros(4)
        self.Y = np.zeros(4)

    def process(self, in_value):
        # 更新输入缓冲区
        self.X[3] = in_value
        # 计算滤波器输出
        # 这个式子和SCIPY的进行对比没有问题
        # https://docs.scipy.org/doc/scipy/reference/generated/scipy.signal.lfilter.html#scipy.signal.lfilter
        self.Y[3] = (self.B[0] * self.X[3] + self.B[1] * self.X[2] + self.B[2] * self.X[1] + self.B[3] * self.X[0] 
                    - self.A[1] * self.Y[2] - self.A[2] * self.Y[1] - self.A[3] * self.Y[0])
        
        # 假设 a(1) = 1
        out_value = self.Y[3]
        # 更新输入和输出缓冲区
        self.X[0] = self.X[1]
        self.X[1] = self.X[2]
        self.X[2] = self.X[3]
        self.Y[0] = self.Y[1]
        self.Y[1] = self.Y[2]
        self.Y[2] = self.Y[3]
        return out_value

# 使用示例
if __name__ == "__main__":
    fs = 1000.0  # 采样频率  250
    cutoff = 50.0  # 截止频率
    b, a = butter3_filter_design(cutoff, fs, order=3)  # 设计滤波器
    print(f"b: {b}, a: {a}")
    # b: [0.09853116 0.29559348 0.29559348 0.09853116], a: [ 1.         -0.57724052  0.42178705 -0.05629724]

    # 创建滤波器实例
    butter_filter = Butter3(b, a)

    # 测试数据
    # 白噪声 长度1000
    noise_signal = np.random.normal(0, 1, 1000)
    # Feed the noise signal into the filter
    # 输出数据记录
    fmt_filtered_output = []
    scipy_filtered_output = []
    for i in range(len(noise_signal)):
        _output = butter_filter.process(noise_signal[i])
        fmt_filtered_output.append(_output)

    # Scipy滤波器
    scipy_filtered_output = lfilter(b, a, noise_signal)

    # 绘制时域比较图像
    plt.figure(figsize=(12, 6))
    plt.plot(noise_signal, label='Noise signal')
    plt.plot(fmt_filtered_output, label='Filtered signal')
    plt.plot(scipy_filtered_output, label='Scipy filtered signal')
    plt.xlabel('Time')
    plt.ylabel('Amplitude')
    plt.title('Butterworth 3rd order filter')
    plt.legend()
    plt.show()

    # FFT计算  频域分析
    noise_fft = np.fft.fft(noise_signal)
    filtered_fft = np.fft.fft(fmt_filtered_output)
    scipy_filtered_fft = np.fft.fft(scipy_filtered_output)

    # 绘制频域图像
    plt.figure(figsize=(12, 6))
    plt.plot(np.abs(noise_fft), label='Noise signal')
    plt.plot(np.abs(filtered_fft), label='Filtered signal')
    plt.plot(np.abs(scipy_filtered_fft), label='Scipy filtered signal')
    plt.xlabel('Frequency')
    plt.ylabel('Magnitude')
    plt.title('Butterworth 3rd order filter')
    plt.legend()
    plt.show()
