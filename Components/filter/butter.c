#include "butter.h"

void butter3_filter_init(Butter3* butter, vfloat b[4], vfloat a[4])
{
    // Butter3* butter = rt_malloc(sizeof(Butter3));//TODO 内存分配
    // if (butter == NULL) {
    //     return NULL;
    // }
    for (uint8_t i = 0; i < 4; i++) {
        butter->B[i] = b[i];//前向通路系数
        butter->A[i] = a[i];//反馈通路系数
        butter->X[i] = butter->Y[i] = 0.0f;//初始化输入输出
    }
}

vfloat butter3_filter_update(Butter3* butter, vfloat in)
{
    vfloat out;
    butter->X[3] = in;//滤波器输入
    /* a(1)*y(n) = b(1)*x(n) + b(2)*x(n-1) + ... + b(nb+1)*x(n-nb) - a(2)*y(n-1) - ... - a(na+1)*y(n-na)  */
    butter->Y[3] = butter->B[0] * butter->X[3] + butter->B[1] * butter->X[2] + butter->B[2] * butter->X[1]
        + butter->B[3] * butter->X[0] - butter->A[1] * butter->Y[2] - butter->A[2] * butter->Y[1] - butter->A[3] * butter->Y[0];
    /* we assume a(1)=1 */
    out = butter->Y[3];//滤波器输出
    /* move X and Y */
    butter->X[0] = butter->X[1];
    butter->X[1] = butter->X[2];
    butter->X[2] = butter->X[3];
    butter->Y[0] = butter->Y[1];
    butter->Y[1] = butter->Y[2];
    butter->Y[2] = butter->Y[3];
    return out;
}
