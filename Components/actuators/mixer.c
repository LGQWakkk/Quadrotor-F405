#include "mixer.h"

// 四旋翼动力分配器
// RP动力优先
// YAW缩放

//4  2
//3  1

// 各个电机扭矩分配系数(一般为-1或者1)
#define M1_ROLL_SCALE   -1.0f
#define M1_PITCH_SCALE  -1.0f
#define M1_YAW_SCALE     1.0f

#define M2_ROLL_SCALE   -1.0f
#define M2_PITCH_SCALE   1.0f
#define M2_YAW_SCALE    -1.0f

#define M3_ROLL_SCALE    1.0f
#define M3_PITCH_SCALE  -1.0f
#define M3_YAW_SCALE    -1.0f

#define M4_ROLL_SCALE    1.0f
#define M4_PITCH_SCALE   1.0f
#define M4_YAW_SCALE     1.0f

// roll, pitch, yaw: 归一化三轴扭矩 -1~1
// thrust: 归一化油门 0~1
// output: 电机归一化推力 0~1

static vfloat upper_limit;
static vfloat lower_limit;
static vfloat limit;
static vfloat rp_abs;
static vfloat rp_scale;
static vfloat yaw_limit;
static vfloat yaw_abs;
static vfloat y_scale;

void mixer_update(vfloat roll, vfloat pitch, vfloat yaw, vfloat thrust, vfloat *output)
{
    roll   = (roll > 1.0f)?   1.0f : ((roll < -1.0f)?   -1.0f : roll);
    pitch  = (pitch > 1.0f)?  1.0f : ((pitch < -1.0f)?  -1.0f : pitch);
    yaw    = (yaw > 1.0f)?    1.0f : ((yaw < -1.0f)?    -1.0f : yaw);
    thrust = (thrust > 1.0f)? 1.0f : ((thrust < 0.0f)?   0.0f : thrust);
    // 计算整体上下余量
    upper_limit = 1.0f - thrust; // 上界
    lower_limit = thrust; // 下界
    limit = (upper_limit > lower_limit)? lower_limit : upper_limit;// 计算最小余量 0-0.5f
    if(limit < 0.01f){ // 基本没有余量 不进行任何控制 直接油门输出
        output[0] = output[1] = output[2] = output[3] = thrust;
        return;
    }
    rp_abs = fabsf(roll) + fabsf(pitch);// 计算RP总体绝对值 0.0-2.0
    if((rp_abs > limit) && (rp_abs > 0.01f)){ // RP超出余量 需要缩放RP
        rp_scale = limit / rp_abs; // 计算缩放比例
        roll *= rp_scale;
        pitch *= rp_scale;
        yaw = 0.0f; // YAW无空间 忽略控制
    }else{ // RP未超出余量 YAW有空间
        yaw_limit = limit - rp_abs; // 计算YAW余量
        yaw_abs = fabsf(yaw); // 计算YAW绝对值
        if((yaw_abs > yaw_limit) && (yaw_abs > 0.01f)){ // YAW超出余量 需要缩放YAW
            y_scale = yaw_limit / yaw_abs; // 计算缩放比例
            yaw *= y_scale;
        }
    }
    // 计算电机转速 动力分配
    output[0] = M1_ROLL_SCALE * roll + M1_PITCH_SCALE * pitch + M1_YAW_SCALE * yaw + thrust;
    output[1] = M2_ROLL_SCALE * roll + M2_PITCH_SCALE * pitch + M2_YAW_SCALE * yaw + thrust;
    output[2] = M3_ROLL_SCALE * roll + M3_PITCH_SCALE * pitch + M3_YAW_SCALE * yaw + thrust;
    output[3] = M4_ROLL_SCALE * roll + M4_PITCH_SCALE * pitch + M4_YAW_SCALE * yaw + thrust;
}
