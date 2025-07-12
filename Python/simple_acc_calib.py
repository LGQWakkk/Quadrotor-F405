# 20250701 Simple Accelerometer Calibration 6-points
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.optimize import leastsq

# Average 6 Samples Data
acc_sample_data = np.array([
    [-2.17336533, -30.65986803, 2055.65586883],
    [-16.86382723,-2050.49910018, 138.30993801],
    [-22.77724455, 12.40831834, -2025.63807239],
    [-48.47670466, 2042.70185963, -14.35172965],
    [2038.07359, -92.3465307, -1.79224155],
    [-2049.97040592, -50.26154769, 20.38932214]
])

x_coords = acc_sample_data[:, 0]
y_coords = acc_sample_data[:, 1]
z_coords = acc_sample_data[:, 2]

magnitudes = np.sqrt(x_coords**2 + y_coords**2 + z_coords**2)
mean_magnitude = np.mean(magnitudes)
variance_magnitude = np.var(magnitudes)

print(f"Average Magnitude: {mean_magnitude}")
print(f"Average Variance: {variance_magnitude}")

def ellipsoid_eq(params, x, y, z):
    x_scale, y_scale, z_scale, x_bias, y_bias, z_bias = params  # 三轴比例和三轴偏置
    return ((x - x_bias)/x_scale)**2 + ((y - y_bias)/y_scale)**2 + ((z - z_bias)/z_scale)**2 - 1

def residuals(params, x, y, z):
    return ellipsoid_eq(params, x, y, z)

# Initial guess for parameters
initial_guess = [2048.0, 2048.0, 2048.0, 0.0, 0.0, 0.0]

result, ier = leastsq(residuals, initial_guess, args=(x_coords, y_coords, z_coords))
x_scale, y_scale, z_scale, x_bias, y_bias, z_bias = result

print(f"Parameters: x_scale = {x_scale}, y_scale = {y_scale}, z_scale = {z_scale}, x_bias = {x_bias}, y_bias = {y_bias}, z_bias = {z_bias}")

fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')

ax.scatter(x_coords, y_coords, z_coords, c='r', marker='.', s=10, label='Data Points')

u = np.linspace(0, 2 * np.pi, 100)
v = np.linspace(0, np.pi, 100)

x = x_scale * np.outer(np.cos(u), np.sin(v)) + x_bias
y = y_scale * np.outer(np.sin(u), np.sin(v)) + y_bias
z = z_scale * np.outer(np.ones_like(u), np.cos(v)) + z_bias

ax.plot_surface(x, y, z, color='b', alpha=0.2, label='Fitted Ellipsoid')

ax.set_xlabel('X Axis')
ax.set_ylabel('Y Axis')
ax.set_zlabel('Z Axis')

ax.legend()
plt.show()

# 20250701 Calib Data:
# Average Magnitude: 1.0053609099515328
# Average Variance: 0.00044733263325932187
# Parameters: x_scale = 1.0026510177994963, y_scale = 1.0050442333500282, z_scale = 1.0075805716618287
# x_bias = 0.028844696088735754, y_bias = 0.006306869620168077, z_bias = 0.021140689403729603
