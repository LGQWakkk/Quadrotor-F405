#pragma once
// 20250702 Wakkk
#include "system.h"
#include "simple_math.h"

typedef struct{
    vfloat kr[3];   // 三轴旋转角度误差增益
    vfloat kw[3];   // 三轴角速度误差增益
    vfloat ki[3];   // 三轴积分误差增益
    vfloat gain[3]; // 三轴增益因子
}so3_controller_param_t;

void so3_controller_set_param(so3_controller_param_t *param); // 参数更新
void so3_controller_control(vfloat *rot_error, vfloat *omega_error, so3_controller_param_t *param, vfloat *output);
void so3_controller_rotation_error(vfloat (*Rc)[3], vfloat (*Rd)[3], vfloat *error);
void so3_controller_omega_error(vfloat (*Rc)[3], vfloat (*Rd)[3], vfloat *omega_c, vfloat *omega_d, vfloat *omega_e);
void so3_controller_update(
    vfloat (*Rc)[3], vfloat (*Rd)[3],  // 姿态控制逻辑
    vfloat *omega_c, vfloat *omega_d,  // 角速度控制逻辑
    vfloat *output                    // 输出控制量
);

extern so3_controller_param_t so3_controller_param;
