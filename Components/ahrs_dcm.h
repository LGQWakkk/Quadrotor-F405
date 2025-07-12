#pragma once

#include "system.h"
#include <stdbool.h>
#include <stdint.h>

#include "simple_math.h"

typedef struct{
    vfloat ahrs_kp;
    vfloat ahrs_ki;
    vfloat ahrs_kp_yaw;
} ahrs_dcm_param_t;

void ahrs_dcm_init(void);
vfloat ahrs_dcm_p_gain(vfloat spin_rate);
bool ahrs_dcm_update(vfloat* gyro, vfloat* acc, vfloat* mag, vfloat dt);
void ahrs_dcm_get_euler(vfloat* roll, vfloat* pitch, vfloat* yaw);
void ahrs_dcm_get_dcm(vfloat (*dcm_out)[3]);
void ahrs_dcm_get_acc_world(vfloat* acc_world_out);
void ahrs_dcm_get_omega_I(vfloat* omega_I_out);
void ahrs_dcm_set_euler_bias(vfloat roll, vfloat pitch, vfloat yaw);
void ahrs_dcm_set_dcm_bias(vfloat (*bias)[3]);
void ahrs_dcm_set_param(ahrs_dcm_param_t* params);  // 参数更新
