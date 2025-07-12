// FMT Butterworth Filter
#pragma once

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>

#include "simple_math.h"
#include "system.h"

typedef struct
{
    vfloat A[4]; //(反馈通路系数)  其中A[0]=1
    vfloat B[4]; //(前向通路系数)
    vfloat X[4]; //历史输入序列 X[3]为最新数据
    vfloat Y[4]; //历史输出序列 IIR滤波器特有 Y[3]为最新输出
}Butter3;

// Butterworth LPF Filter
// 三阶巴特沃斯滤波器
void butter3_filter_init(Butter3* butter, vfloat b[4], vfloat a[4]);
vfloat butter3_filter_update(Butter3* butter, vfloat in);
