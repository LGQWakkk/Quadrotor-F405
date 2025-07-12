#include "ahrs_dcm.h"
#include "simple_math.h"
// 机体前方X 机体右方Y 机体下方Z
// 20250711 Wakkk

#define AHRS_DCM_KP 0.05f
#define AHRS_DCM_KP_YAW 0.05f
#define AHRS_DCM_KI 0.001f
#define AHRS_SPIN_RATE_LIMIT 20.0f
#define AHRS_ACC_NORM_MIN 9.0f  // 进行加速度计补偿的最小模值(m/s2)
#define AHRS_ACC_NORM_MAX 11.0f  // 进行加速度计补偿的最大模值(m/s2)

static vfloat dcm_bias[3][3]; // 姿态偏置

static vfloat omega[3];
static vfloat omega_P[3];
static vfloat omega_P_yaw[3];
static vfloat omega_I[3];
static vfloat dcm[3][3];
static vfloat spin_rate; // rad/s

static vfloat ahrs_kp;
static vfloat ahrs_ki;
static vfloat ahrs_kp_yaw;

static vfloat acc_world[3];
static vfloat acc_body[3];
static vfloat g_world[3]; // NED坐标系重力加速度单位矢量

// 更新AHRS DCM参数
void ahrs_dcm_set_param(ahrs_dcm_param_t* params)
{
    ahrs_kp = params->ahrs_kp;
    ahrs_ki = params->ahrs_ki;
    ahrs_kp_yaw = params->ahrs_kp_yaw;
}

// 设置姿态偏置欧拉角
static vfloat dcm_bias_tmp_1[3][3];
void ahrs_dcm_set_euler_bias(vfloat roll, vfloat pitch, vfloat yaw)
{
    Matrix3FromEuler(roll, pitch, yaw, dcm_bias_tmp_1); // 欧拉角构建旋转矩阵
    Matrix3Transpose(dcm_bias_tmp_1, dcm_bias); // 需要转置
}

// 设置姿态偏置旋转矩阵
static vfloat dcm_bias_tmp_2[3][3];
void ahrs_dcm_set_dcm_bias(vfloat (*bias)[3])
{
    Matrix3Normalize(bias, dcm_bias_tmp_2); // 进一步归一化
    Matrix3Transpose(dcm_bias_tmp_2, dcm_bias); // 需要转置
}

void ahrs_dcm_init(void)
{
    Matrix3Identity(dcm);
    Vector3Zero(omega);
    Vector3Zero(omega_P);
    Vector3Zero(omega_P_yaw);
    Vector3Zero(omega_I);
    Vector3Zero(acc_world);
    Vector3Zero(acc_body);

    // 初始化姿态偏置
    Matrix3Identity(dcm_bias);

    spin_rate = 0.0f;

    // ahrs_kp = AHRS_DCM_KP;
    // ahrs_ki = AHRS_DCM_KI;
    // ahrs_kp_yaw = AHRS_DCM_KP_YAW;
    ahrs_kp = 15.0f;
    ahrs_ki = 0.0f;
    ahrs_kp_yaw = 0.0f;

    g_world[0] = 0.0f; g_world[1] = 0.0f; g_world[2] = 1.0f;
}

vfloat ahrs_dcm_p_gain(vfloat spin_rate)
{
    if (spin_rate < DEG2RAD(50)){
        return 1.0f;
    }
    if (spin_rate > DEG2RAD(500)){
        return 10.0f;
    }
    return spin_rate/DEG2RAD(50);
}

// gyro: rad/s
// acc: m/s^2 (None if not used)
// mag: Gauss (None if not used)

static vfloat delta_rotate[3];
static vfloat tmp_dcm[3][3];
static vfloat acc_world_norm[3];
static vfloat error[3];
static vfloat omega_I_tmp[3];

bool ahrs_dcm_update(vfloat* gyro, vfloat* acc, vfloat* mag, vfloat dt)
{
    if(dt <= 0.0f)return false;
    if(gyro == NULL)return false;
    Vector3Copy(gyro, omega);
    
    Vector3Add(omega, omega_I, omega); // 偏置估计补偿
    spin_rate = Vector3Length(omega);

    Vector3Add(delta_rotate, omega, delta_rotate); // omega + omega_I
    Vector3Add(delta_rotate, omega_P, delta_rotate); // omega + omega_I + omega_P
    Vector3Add(delta_rotate, omega_P_yaw, delta_rotate); // omega + omega_I + omega_P + omega_yaw_P
    Vector3Scale(delta_rotate, dt, delta_rotate); // dt
    Matrix3Rotate(dcm, delta_rotate, tmp_dcm);
    Matrix3Copy(tmp_dcm, dcm);
    Matrix3Normalize(dcm, dcm); // 归一化
    
    if(acc == NULL) return true; // Use ACC
    vfloat acc_norm = Vector3Length(acc);
    // slog_dma("acc_norm:%f\r\n", acc_norm);
    if(acc_norm < AHRS_ACC_NORM_MIN || acc_norm > AHRS_ACC_NORM_MAX)return true;// 不进行ACC补偿

    Vector3Copy(acc, acc_body);
    Matrix3MulVector(dcm, acc_body, acc_world);
    Vector3Normalize(acc_world, acc_world_norm);
    Vector3CrossProduct(acc_world_norm, g_world, error);
    vfloat error_length = Vector3Length(error);
    // slog_dma("error_length:%f\r\n", error_length);// 这个数值巨小 0.00006
    Vector3Copy(error, omega_P); // omega_P = error
    Vector3Scale(omega_P, ahrs_dcm_p_gain(spin_rate) * ahrs_kp, omega_P); // omega_P = error * kp * p_gain
    if(spin_rate < DEG2RAD(AHRS_SPIN_RATE_LIMIT)){
        Vector3Copy(error, omega_I_tmp); // omega_I_tmp = error
        Vector3Scale(omega_I_tmp, ahrs_ki * dt, omega_I_tmp); // omega_I_tmp = error * ki * dt
        Vector3Add(omega_I, omega_I_tmp, omega_I); // omega_I = omega_I + omega_I_tmp
    }
    return true;
}

void ahrs_dcm_get_omega_I(vfloat* omega_I_out)
{
    Vector3Copy(omega_I, omega_I_out);
}

static vfloat dcm_tmp[3][3];
void ahrs_dcm_get_euler(vfloat* roll, vfloat* pitch, vfloat* yaw)
{
    Matrix3MulMatrix(dcm_bias, dcm, dcm_tmp); // 施加偏置 使用偏置左乘DCM
    Matrix3ToEuler(dcm_tmp, roll, pitch, yaw);
}

void ahrs_dcm_get_dcm(vfloat (*dcm_out)[3])
{
    Matrix3MulMatrix(dcm_bias, dcm, dcm_out); // 施加偏置 使用偏置左乘DCM
    // Matrix3Copy(dcm, dcm_out);  // 不施加偏置
}

void ahrs_dcm_get_acc_world(vfloat* acc_world_out)
{
    Vector3Copy(acc_world, acc_world_out);
}
