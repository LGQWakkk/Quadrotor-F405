#pragma once
// 四旋翼电机动力分配器 20250710 Wakkk
#include "simple_math.h"

void mixer_update(vfloat roll, vfloat pitch, vfloat yaw, vfloat thrust, vfloat *output);
