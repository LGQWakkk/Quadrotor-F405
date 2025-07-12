// 20250702 Wakkk
#include "so3_controller.h"

// 当只有旋转误差增益为1时 最大输出控制量为1
so3_controller_param_t so3_controller_param;

#define INTEGRAL_LIMIT (0.25f)

// 更新SO3控制器参数
void so3_controller_set_param(so3_controller_param_t *param)
{
    // so3_controller_param = *param; ???
    Vector3Copy(param->kr, so3_controller_param.kr);
    Vector3Copy(param->kw, so3_controller_param.kw);
    Vector3Copy(param->ki, so3_controller_param.ki);
    Vector3Copy(param->gain, so3_controller_param.gain);
}

static vfloat rot_error[3];
static vfloat omega_error[3];
static vfloat rot_error_int[3]; // 三轴角度误差积分

void so3_controller_update(
    vfloat (*Rc)[3], vfloat (*Rd)[3],  // 姿态控制逻辑
    vfloat *omega_c, vfloat *omega_d,  // 角速度控制逻辑
    vfloat *output                    // 输出控制量
)
{
    // 1.计算姿态误差
    so3_controller_rotation_error(Rc, Rd, rot_error);
    // 2.计算角速度误差
    so3_controller_omega_error(Rc, Rd, omega_c, omega_d, omega_error);
    // 3.计算控制量
    so3_controller_control(rot_error, omega_error, &so3_controller_param, output);
}

// _rot_error: 三轴旋转误差
// _omega_error: 三轴角速度误差
// param: 控制器参数
// output: 输出控制量
void so3_controller_control(
    vfloat *_rot_error, vfloat *_omega_error, so3_controller_param_t *param, vfloat *output)
{
    rot_error_int[0] += _rot_error[0];
    rot_error_int[1] += _rot_error[1];
    rot_error_int[2] += _rot_error[2];

    rot_error_int[0] = (rot_error_int[0] > INTEGRAL_LIMIT)? INTEGRAL_LIMIT : rot_error_int[0];
    rot_error_int[1] = (rot_error_int[1] > INTEGRAL_LIMIT)? INTEGRAL_LIMIT : rot_error_int[1];
    rot_error_int[2] = (rot_error_int[2] > INTEGRAL_LIMIT)? INTEGRAL_LIMIT : rot_error_int[2];

    rot_error_int[0] = (rot_error_int[0] < -INTEGRAL_LIMIT)? -INTEGRAL_LIMIT : rot_error_int[0];
    rot_error_int[1] = (rot_error_int[1] < -INTEGRAL_LIMIT)? -INTEGRAL_LIMIT : rot_error_int[1];
    rot_error_int[2] = (rot_error_int[2] < -INTEGRAL_LIMIT)? -INTEGRAL_LIMIT : rot_error_int[2];

    output[0] = (param->kr[0])*_rot_error[0] + (param->kw[0])*_omega_error[0] + (param->ki[0])*rot_error_int[0];
    output[1] = (param->kr[1])*_rot_error[1] + (param->kw[1])*_omega_error[1] + (param->ki[1])*rot_error_int[1];
    output[2] = (param->kr[2])*_rot_error[2] + (param->kw[2])*_omega_error[2] + (param->ki[2])*rot_error_int[2];

    output[0] *= (param->gain[0]);
    output[1] *= (param->gain[1]);
    output[2] *= (param->gain[2]);
}

void so3_controller_rotation_error(vfloat (*Rc)[3], vfloat (*Rd)[3], vfloat *error)
{
    vfloat RdT[3][3];
    vfloat RdTRc[3][3];
    vfloat RcT[3][3];
    vfloat RcTRd[3][3];
    vfloat Rerror[3][3];
    Matrix3Transpose(Rc, RcT);
    Matrix3Transpose(Rd, RdT);
    Matrix3MulMatrix(RdT, Rc, RdTRc);
    Matrix3MulMatrix(RcT, Rd, RcTRd);
    Matrix3Sub(RdTRc, RcTRd, Rerror);
    vee(Rerror, error);
    Vector3Scale(error, 0.5f, error);
}

void so3_controller_omega_error(
    vfloat (*Rc)[3], vfloat (*Rd)[3], 
    vfloat *omega_c, vfloat *omega_d, vfloat *omega_e)
{
    vfloat RcT[3][3];
    vfloat omega_d_world[3]; // 世界坐标系 目标角速度
    vfloat omega_d_body[3]; // 当前机体坐标系 目标角速度
    Matrix3Transpose(Rc, RcT);
    Matrix3MulVector(Rd, omega_d, omega_d_world);
    Matrix3MulVector(RcT, omega_d_world, omega_d_body);
    Vector3Sub(omega_c, omega_d_body, omega_e);
}
