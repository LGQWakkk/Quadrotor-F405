#pragma once

#include <math.h>

typedef volatile float vfloat;

// 20250702 Wakkk
#define EPSILON 0.00001f

#define q1 (q[0])
#define q2 (q[1])
#define q3 (q[2])
#define q4 (q[3])

#define qw (q[0])
#define qx (q[1])
#define qy (q[2])
#define qz (q[3])

#define M_PI   3.14159265358979323846f
#define RAD (M_PI / 180.0f)
#define DEG2RAD(angle) ((angle) * RAD)
#define RAD2DEG(angle) ((angle) / RAD)

vfloat simple_sqrt(vfloat x);
vfloat simple_cos(vfloat x);
vfloat simple_sin(vfloat x);
vfloat simple_atan2(vfloat y, vfloat x);
vfloat simple_asin(vfloat x);

void  Vector3CrossProduct(vfloat* a, vfloat* b, vfloat* result);
vfloat Vector3DotProduct(vfloat* a, vfloat* b);
void  Vector3Add(vfloat* a, vfloat* b, vfloat* result);
void  Vector3Sub(vfloat* a, vfloat* b, vfloat* result);
void  Vector3Scale(vfloat* a, vfloat b, vfloat* result);
void  Vector3Normalize(vfloat* a, vfloat* result);
vfloat Vector3Length(vfloat* a);
vfloat Vector3LengthSquared(vfloat* a);
void  Vector3Zero(vfloat* a);
void  Vector3Copy(vfloat* a, vfloat* result);

vfloat Vector2CrossProduct(vfloat* a, vfloat* b);
vfloat Vector2DotProduct(vfloat* a, vfloat* b);
void  Vector2Add(vfloat* a, vfloat* b, vfloat* result);
void  Vector2Sub(vfloat* a, vfloat* b, vfloat* result);
void  Vector2Scale(vfloat* a, vfloat b, vfloat* result);
void  Vector2Normalize(vfloat* a, vfloat* result);
vfloat Vector2Length(vfloat* a);
vfloat Vector2LengthSquared(vfloat* a);
void  Vector2Zero(vfloat* a);
void  Vector2Copy(vfloat* a, vfloat* result);

void  Matrix3Add(vfloat (*ma)[3], vfloat (*mb)[3], vfloat (*mo)[3]);
void  Matrix3Sub(vfloat (*ma)[3], vfloat (*mb)[3], vfloat (*mo)[3]);
void  Matrix3FromEuler(vfloat roll, vfloat pitch, vfloat yaw, vfloat (*result)[3]);
void  Matrix3ToEuler(vfloat (*m)[3], vfloat* roll, vfloat* pitch, vfloat* yaw);
void  Matrix3Rotate(vfloat (*mi)[3], vfloat* v, vfloat (*mo)[3]);
void  Matrix3Normalize(vfloat (*mi)[3], vfloat (*mo)[3]);
void  Matrix3MulVector(vfloat (*m)[3], vfloat* v, vfloat* result);
void  Matrix3MulMatrix(vfloat (*ma)[3], vfloat (*mb)[3], vfloat (*mo)[3]);
void  Matrix3Zero(vfloat (*m)[3]);
void  Matrix3Copy(vfloat (*ma)[3], vfloat (*mo)[3]);
void  Matrix3Identity(vfloat (*m)[3]);
void  Matrix3Transpose(vfloat (*ma)[3], vfloat (*mo)[3]);

void  QuaternionToMatrix(vfloat* q, vfloat (*m)[3]);
void  QuaternionFromMatrix(vfloat (*m)[3], vfloat* q);
void  QuaternionFromEuler(vfloat roll, vfloat pitch, vfloat yaw, vfloat* q);
void  QuaternionToEuler(vfloat* q, vfloat* roll, vfloat* pitch, vfloat* yaw);
vfloat QuaternionLength(vfloat* q);
vfloat QuaternionLengthSquared(vfloat* q);
vfloat QuaternionInverse(vfloat* q, vfloat* result);
void  QuaternionNormalize(vfloat* q, vfloat* result);
void  QuaternionScale(vfloat* q, vfloat b, vfloat* result);
void  QuaternionIdentity(vfloat* q);
void  QuaternionMulQuaternion(vfloat* qa, vfloat* qb, vfloat* result);

void vee(vfloat (*m)[3], vfloat* v);
void hat(vfloat* v, vfloat (*m)[3]);
