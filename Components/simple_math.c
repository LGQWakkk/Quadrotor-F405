#include "simple_math.h"

#include <arm_math.h> 

// 1.16.2 CMSIS-DSP
// https://arm-software.github.io/CMSIS-DSP/latest/index.html

////////////////////////////////////////////Basic//////////////////////////////////////////////

// 支持使用自定义的sqrt函数
vfloat simple_sqrt(vfloat x)
{
    // return sqrtf(x);
    vfloat result;
    arm_sqrt_f32(x, &result);
    return result;
}

vfloat simple_cos(vfloat x)
{
    // return cosf(x);
    return arm_cos_f32(x);
}

vfloat simple_sin(vfloat x)
{
    // return sinf(x);
    return arm_sin_f32(x);
}

vfloat simple_atan2(vfloat y, vfloat x)
{
    // return atan2f(y, x);
    vfloat result;
    arm_atan2_f32(y, x, &result);
    return result;
}

vfloat simple_asin(vfloat x)
{
    return asinf(x);
}

////////////////////////////////////////////Vector3////////////////////////////////////////////
// 向量叉乘
void Vector3CrossProduct(vfloat* a, vfloat* b, vfloat* result)
{
    result[0] = a[1] * b[2] - a[2] * b[1];
    result[1] = a[2] * b[0] - a[0] * b[2];
    result[2] = a[0] * b[1] - a[1] * b[0];
}

// 向量点积
vfloat Vector3DotProduct(vfloat* a, vfloat* b)
{
    return (a[0] * b[0] + a[1] * b[1] + a[2] * b[2]);
}

// 向量相加
void Vector3Add(vfloat* a, vfloat* b, vfloat* result)
{
    result[0] = a[0] + b[0];
    result[1] = a[1] + b[1];
    result[2] = a[2] + b[2];
}

// 向量相减
void Vector3Sub(vfloat* a, vfloat* b, vfloat* result)
{
    result[0] = a[0] - b[0];
    result[1] = a[1] - b[1];
    result[2] = a[2] - b[2];
}

// 向量缩放 支持原地操作
void Vector3Scale(vfloat* a, vfloat b, vfloat* result)
{
    result[0] = a[0] * b;
    result[1] = a[1] * b;
    result[2] = a[2] * b;
}

// 向量单位化 支持原地操作
void Vector3Normalize(vfloat* a, vfloat* result)
{
    vfloat length = Vector3Length(a);
    if (length > EPSILON){
        Vector3Scale(a, 1.0f / length, result);
    }else{
        result[0] = result[1] = result[2] = 0.0f;
    }
}

// 向量模
vfloat Vector3Length(vfloat* a)
{
    return simple_sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
}

// 向量模的平方
vfloat Vector3LengthSquared(vfloat* a)
{
    return (a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
}

// 向量复位
void Vector3Zero(vfloat* a)
{
    a[0] = a[1] = a[2] = 0.0f;
}

// 向量复制
void Vector3Copy(vfloat* a, vfloat* result)
{
    result[0] = a[0];
    result[1] = a[1];
    result[2] = a[2];
}

////////////////////////////////////////////Vector2////////////////////////////////////////////
// 向量叉乘
vfloat Vector2CrossProduct(vfloat* a, vfloat* b)
{
    return (a[0] * b[1] - a[1] * b[0]);
}

// 向量点积
vfloat Vector2DotProduct(vfloat* a, vfloat* b)
{
    return (a[0] * b[0] + a[1] * b[1]);
}

// 向量相加
void Vector2Add(vfloat* a, vfloat* b, vfloat* result)
{
    result[0] = a[0] + b[0];
    result[1] = a[1] + b[1];
}

// 向量相减
void Vector2Sub(vfloat* a, vfloat* b, vfloat* result)
{
    result[0] = a[0] - b[0];
    result[1] = a[1] - b[1];
}

// 向量缩放
void Vector2Scale(vfloat* a, vfloat b, vfloat* result)
{
    result[0] = a[0] * b;
    result[1] = a[1] * b;
}

// 向量单位化
void Vector2Normalize(vfloat* a, vfloat* result)
{
    vfloat length = Vector2Length(a);
    if (length > EPSILON){
        Vector2Scale(a, 1.0f / length, result);
    }else{
        result[0] = result[1] = 0.0f;
    }
}

// 向量模
vfloat Vector2Length(vfloat* a)
{
    return simple_sqrt(a[0] * a[0] + a[1] * a[1]);
}

// 向量模的平方
vfloat Vector2LengthSquared(vfloat* a)
{
    return (a[0] * a[0] + a[1] * a[1]);
}

void Vector2Zero(vfloat* a)
{
    a[0] = a[1] = 0.0f;
}

// 向量复制
void Vector2Copy(vfloat* a, vfloat* result)
{
    result[0] = a[0];
    result[1] = a[1];
}

////////////////////////////////////////////Matrix3////////////////////////////////////////////

// TODO
void Matrix3Add(vfloat (*ma)[3], vfloat (*mb)[3], vfloat (*mo)[3])
{
    mo[0][0] = ma[0][0] + mb[0][0];
    mo[0][1] = ma[0][1] + mb[0][1];
    mo[0][2] = ma[0][2] + mb[0][2];
    mo[1][0] = ma[1][0] + mb[1][0];
    mo[1][1] = ma[1][1] + mb[1][1];
    mo[1][2] = ma[1][2] + mb[1][2];
    mo[2][0] = ma[2][0] + mb[2][0];
    mo[2][1] = ma[2][1] + mb[2][1];
    mo[2][2] = ma[2][2] + mb[2][2];
}

// TODO
void Matrix3Sub(vfloat (*ma)[3], vfloat (*mb)[3], vfloat (*mo)[3])
{
    mo[0][0] = ma[0][0] - mb[0][0];
    mo[0][1] = ma[0][1] - mb[0][1];
    mo[0][2] = ma[0][2] - mb[0][2];
    mo[1][0] = ma[1][0] - mb[1][0];
    mo[1][1] = ma[1][1] - mb[1][1];
    mo[1][2] = ma[1][2] - mb[1][2];
    mo[2][0] = ma[2][0] - mb[2][0];
    mo[2][1] = ma[2][1] - mb[2][1];
    mo[2][2] = ma[2][2] - mb[2][2];
}

// 使用欧拉角设置旋转矩阵
void Matrix3FromEuler(vfloat roll, vfloat pitch, vfloat yaw, vfloat (*result)[3])
{
    vfloat cp = simple_cos(pitch);
    vfloat sp = simple_sin(pitch);
    vfloat sr = simple_sin(roll);
    vfloat cr = simple_cos(roll);
    vfloat sy = simple_sin(yaw);
    vfloat cy = simple_cos(yaw);

    result[0][0] = cp * cy;
    result[0][1] = (sr * sp * cy) - (cr * sy);
    result[0][2] = (cr * sp * cy) + (sr * sy);
    result[1][0] = cp * sy;
    result[1][1] = (sr * sp * sy) + (cr * cy);
    result[1][2] = (cr * sp * sy) - (sr * cy);
    result[2][0] = -sp;
    result[2][1] = sr * cp;
    result[2][2] = cr * cp;
}

// 旋转矩阵转换为欧拉角
void Matrix3ToEuler(vfloat (*m)[3], vfloat* roll, vfloat* pitch, vfloat* yaw)
{
    *pitch = -simple_asin (m[2][0]);
    *roll  =  simple_atan2(m[2][1], m[2][2]);
    *yaw   =  simple_atan2(m[1][0], m[0][0]);
}

// 使用向量对矩阵进行旋转
// mi: 输入矩阵 mo: 输出矩阵 v: 旋转向量
void Matrix3Rotate(vfloat (*mi)[3], vfloat* v, vfloat (*mo)[3])
{
    mo[0][0] = mi[0][0] + (mi[0][1] * v[2] - mi[0][2] * v[1]);
    mo[0][1] = mi[0][1] + (mi[0][2] * v[0] - mi[0][0] * v[2]);
    mo[0][2] = mi[0][2] + (mi[0][0] * v[1] - mi[0][1] * v[0]);
    mo[1][0] = mi[1][0] + (mi[1][1] * v[2] - mi[1][2] * v[1]);
    mo[1][1] = mi[1][1] + (mi[1][2] * v[0] - mi[1][0] * v[2]);
    mo[1][2] = mi[1][2] + (mi[1][0] * v[1] - mi[1][1] * v[0]);
    mo[2][0] = mi[2][0] + (mi[2][1] * v[2] - mi[2][2] * v[1]);
    mo[2][1] = mi[2][1] + (mi[2][2] * v[0] - mi[2][0] * v[2]);
    mo[2][2] = mi[2][2] + (mi[2][0] * v[1] - mi[2][1] * v[0]);
}

// 旋转矩阵归一化 支持原地操作
void Matrix3Normalize(vfloat (*mi)[3], vfloat (*mo)[3])
{
    vfloat error = 0.5 * Vector3DotProduct(mi[0], mi[1]);// 0 Y分量正交误差
    vfloat t0[3],t1[3],t2[3];
    Vector3Copy(mi[1], t0);// b
    Vector3Copy(mi[0], t1);// a
    Vector3Scale(t0, -error, t0);// -b*error
    Vector3Scale(t1, -error, t1);// -a*error
    Vector3Add(t0, mi[0], t0);// a-b*error
    Vector3Add(t1, mi[1], t1);// b-a*error
    Vector3CrossProduct(t0, t1, t2);// t2 = t0 % t1
    // 向量单位化
    Vector3Normalize(t0, t0);
    Vector3Normalize(t1, t1);
    Vector3Normalize(t2, t2);
    // 更新矩阵
    Vector3Copy(t0, mo[0]);
    Vector3Copy(t1, mo[1]);
    Vector3Copy(t2, mo[2]);
}

// 矩阵和向量相乘
void Matrix3MulVector(vfloat (*m)[3], vfloat* v, vfloat* result)
{
    result[0] = m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2];
    result[1] = m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2];
    result[2] = m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2];
}

// 矩阵和矩阵相乘
void Matrix3MulMatrix(vfloat (*ma)[3], vfloat (*mb)[3], vfloat (*mo)[3])
{
    mo[0][0] = ma[0][0] * mb[0][0] + ma[0][1] * mb[1][0] + ma[0][2] * mb[2][0];
    mo[0][1] = ma[0][0] * mb[0][1] + ma[0][1] * mb[1][1] + ma[0][2] * mb[2][1];
    mo[0][2] = ma[0][0] * mb[0][2] + ma[0][1] * mb[1][2] + ma[0][2] * mb[2][2];
    mo[1][0] = ma[1][0] * mb[0][0] + ma[1][1] * mb[1][0] + ma[1][2] * mb[2][0];
    mo[1][1] = ma[1][0] * mb[0][1] + ma[1][1] * mb[1][1] + ma[1][2] * mb[2][1];
    mo[1][2] = ma[1][0] * mb[0][2] + ma[1][1] * mb[1][2] + ma[1][2] * mb[2][2];
    mo[2][0] = ma[2][0] * mb[0][0] + ma[2][1] * mb[1][0] + ma[2][2] * mb[2][0];
    mo[2][1] = ma[2][0] * mb[0][1] + ma[2][1] * mb[1][1] + ma[2][2] * mb[2][1];
    mo[2][2] = ma[2][0] * mb[0][2] + ma[2][1] * mb[1][2] + ma[2][2] * mb[2][2];
}

// 矩阵复位
void Matrix3Zero(vfloat (*m)[3])
{   // TODO使用更高效方法
    m[0][0] = m[0][1] = m[0][2] = 0.0f;
    m[1][0] = m[1][1] = m[1][2] = 0.0f;
    m[2][0] = m[2][1] = m[2][2] = 0.0f;
}

// 矩阵复制
void Matrix3Copy(vfloat (*ma)[3], vfloat (*mo)[3])
{   // TODO使用更高效方法
    Vector3Copy(ma[0], mo[0]);
    Vector3Copy(ma[1], mo[1]);
    Vector3Copy(ma[2], mo[2]);
}

// 矩阵设置为单位矩阵
void Matrix3Identity(vfloat (*m)[3])
{
    Matrix3Zero(m);
    m[0][0] = m[1][1] = m[2][2] = 1.0f;
}

// 矩阵转置
void Matrix3Transpose(vfloat (*ma)[3], vfloat (*mo)[3])
{
    mo[0][0] = ma[0][0];
    mo[0][1] = ma[1][0];
    mo[0][2] = ma[2][0];
    mo[1][0] = ma[0][1];
    mo[1][1] = ma[1][1];
    mo[1][2] = ma[2][1];
    mo[2][0] = ma[0][2];
    mo[2][1] = ma[1][2];
    mo[2][2] = ma[2][2];
}

////////////////////////////////////////////Quaternion////////////////////////////////////////////

// 四元数转换为旋转矩阵
void QuaternionToMatrix(vfloat* q, vfloat (*m)[3])
{
    vfloat q3q3 = q3 * q3;
    vfloat q3q4 = q3 * q4;
    vfloat q2q2 = q2 * q2;
    vfloat q2q3 = q2 * q3;
    vfloat q2q4 = q2 * q4;
    vfloat q1q2 = q1 * q2;
    vfloat q1q3 = q1 * q3;
    vfloat q1q4 = q1 * q4;
    vfloat q4q4 = q4 * q4;
    m[0][0] = 1.0f-2.0f*(q3q3 + q4q4);
    m[0][1] = 2.0f*(q2q3 - q1q4);
    m[0][2] = 2.0f*(q2q4 + q1q3);
    m[1][0] = 2.0f*(q2q3 + q1q4);
    m[1][1] = 1.0f-2.0f*(q2q2 + q4q4);
    m[1][2] = 2.0f*(q3q4 - q1q2);
    m[2][0] = 2.0f*(q2q4 - q1q3);
    m[2][1] = 2.0f*(q3q4 + q1q2);
    m[2][2] = 1.0f-2.0f*(q2q2 + q3q3);
}

// 旋转矩阵转换为四元数
void QuaternionFromMatrix(vfloat (*m)[3], vfloat* q)
{
    vfloat m00 = m[0][0];
    vfloat m11 = m[1][1];
    vfloat m22 = m[2][2];
    vfloat m10 = m[1][0];
    vfloat m01 = m[0][1];
    vfloat m20 = m[2][0];
    vfloat m02 = m[0][2];
    vfloat m21 = m[2][1];
    vfloat m12 = m[1][2];
    vfloat tr = m00 + m11 + m22;
    if (tr > 0) {
        vfloat S = simple_sqrt(tr+1) * 2;
        qw = 0.25f * S;
        qx = (m21 - m12) / S;
        qy = (m02 - m20) / S;
        qz = (m10 - m01) / S;
    } else if ((m00 > m11) && (m00 > m22)) {
        vfloat S = simple_sqrt(1.0f + m00 - m11 - m22) * 2.0f;
        qw = (m21 - m12) / S;
        qx = 0.25f * S;
        qy = (m01 + m10) / S;
        qz = (m02 + m20) / S;
    } else if (m11 > m22) {
        vfloat S = simple_sqrt(1.0f + m11 - m00 - m22) * 2.0f;
        qw = (m02 - m20) / S;
        qx = (m01 + m10) / S;
        qy = 0.25f * S;
        qz = (m12 + m21) / S;
    } else {
        vfloat S = simple_sqrt(1.0f + m22 - m00 - m11) * 2.0f;
        qw = (m10 - m01) / S;
        qx = (m02 + m20) / S;
        qy = (m12 + m21) / S;
        qz = 0.25f * S;
    }
}

// 欧拉角转换为四元数
void QuaternionFromEuler(vfloat roll, vfloat pitch, vfloat yaw, vfloat* q)
{
    vfloat cr2 = simple_cos(roll*0.5);
    vfloat cp2 = simple_cos(pitch*0.5);
    vfloat cy2 = simple_cos(yaw*0.5);
    vfloat sr2 = simple_sin(roll*0.5);
    vfloat sp2 = simple_sin(pitch*0.5);
    vfloat sy2 = simple_sin(yaw*0.5);
    q1 = cr2*cp2*cy2 + sr2*sp2*sy2;
    q2 = sr2*cp2*cy2 - cr2*sp2*sy2;
    q3 = cr2*sp2*cy2 + sr2*cp2*sy2;
    q4 = cr2*cp2*sy2 - sr2*sp2*cy2;
}

// 四元数转换为欧拉角
void QuaternionToEuler(vfloat* q, vfloat* roll, vfloat* pitch, vfloat* yaw)
{
    *roll  = simple_atan2(2.0f*(q1*q2 + q3*q4), 1.0f - 2.0f*(q2*q2 + q3*q3));
    *pitch = simple_asin (2.0f*(q1*q3 - q4*q2));
    *yaw   = simple_atan2(2.0f*(q1*q4 + q2*q3), 1.0f - 2.0f*(q3*q3 + q4*q4));
}

// 获取四元数模值
vfloat QuaternionLength(vfloat* q)
{
    return simple_sqrt(q1*q1 + q2*q2 + q3*q3 + q4*q4);
}

// 获取四元数模值平方
vfloat QuaternionLengthSquared(vfloat* q)
{
    return q1*q1 + q2*q2 + q3*q3 + q4*q4;
}

// 获取四元数对应的逆旋转 支持原地操作
vfloat QuaternionInverse(vfloat* q, vfloat* result)
{
    result[0] = q[0];
    result[1] = -q[1];
    result[2] = -q[2];
    result[3] = -q[3];
}

// 四元数单位化 支持原地操作
void QuaternionNormalize(vfloat* q, vfloat* result)
{
    vfloat length = QuaternionLength(q);
    if(length > EPSILON){
        QuaternionScale(q, 1.0f / length, result);
    }else{ // 数值错误 复位为单位四元数
        result[0] = 1.0f;
        result[1] = result[2] = result[3] = 0.0f;
    }
}

// 四元数缩放 支持原地操作
void QuaternionScale(vfloat* q, vfloat b, vfloat* result)
{
    result[0] = q[0] * b;
    result[1] = q[1] * b;
    result[2] = q[2] * b;
    result[3] = q[3] * b;
}

// 四元数复位为单位四元数
void QuaternionIdentity(vfloat* q)
{
    q1 = 1.0f;
    q2 = q3 = q4 = 0.0f;
}

// 四元数乘以四元数
void QuaternionMulQuaternion(vfloat* qa, vfloat* qb, vfloat* result)
{
    result[0] = qa[0]*qb[0] - qa[1]*qb[1] - qa[2]*qb[2] - qa[3]*qb[3];
    result[1] = qa[0]*qb[1] + qa[1]*qb[0] + qa[2]*qb[3] - qa[3]*qb[2];
    result[2] = qa[0]*qb[2] - qa[1]*qb[3] + qa[2]*qb[0] + qa[3]*qb[1];
    result[3] = qa[0]*qb[3] + qa[1]*qb[2] - qa[2]*qb[1] + qa[3]*qb[0];
}

void vee(vfloat (*m)[3], vfloat* v)
{
    v[0] = m[2][1];
    v[1] = m[0][2];
    v[2] = m[1][0];
}

void hat(vfloat* v, vfloat (*m)[3])
{
    Matrix3Zero(m);
    m[1][0] =  v[2];
    m[0][1] = -v[2];
    m[2][0] = -v[1];
    m[0][2] =  v[1];
    m[2][1] =  v[0];
    m[1][2] = -v[0];
}
