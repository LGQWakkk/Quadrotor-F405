#include "system.h"
#include "dshot.h"
#include "icm42688p.h"
#include "spl06.h"
#include "qmc5883l.h"
#include "crsf.h"
#include "si24r1.h"
#include "param.h"

#include "ahrs_dcm.h"
#include "simple_math.h"
#include "so3_controller.h"
#include "butter.h"
#include "mixer.h"

/////////////////////////////////////////DEBUG/////////////////////////////////////////
uint8_t log_buffer[128];
void slog(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    vsnprintf((char*)log_buffer, sizeof(log_buffer), fmt, args);
    va_end(args);
    debug_uart_tx(log_buffer, strlen((char*)log_buffer));
}
void slog_dma(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    vsnprintf((char*)log_buffer, sizeof(log_buffer), fmt, args);
    va_end(args);
    debug_uart_tx_dma((uint32_t)log_buffer, strlen((char*)log_buffer));
}
void debug_uart_tx(uint8_t *buffer, uint32_t len)
{
    uint32_t index = 0;
    uint8_t *p = buffer;
    for (index = 0; index < len; index++){
        while (!LL_USART_IsActiveFlag_TXE(USART1));
        LL_USART_TransmitData8(USART1, *p++);
        }
    while(!LL_USART_IsActiveFlag_TC(USART1));
}
void debug_uart_tx_dma(uint32_t buffer_addr, uint32_t len)
{
    debug_uart_configure_tx_dma(buffer_addr, len);
    debug_uart_tx_dma_enable();
}
void debug_uart_tx_dma_cplt_callback(void)
{
    // Nothing to do here
}
void debug_uart_tx_dma_enable(void)
{
    LL_USART_EnableDMAReq_TX(USART1);
    LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_7);
}
void debug_uart_tx_dma_disable(void)
{
    LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_7);
    // LL_USART_DisableDMAReq_TX(USART1);
}
void debug_uart_configure_tx_dma(uint32_t buffer_addr, uint32_t len)
{
    LL_DMA_ConfigAddresses(DMA2, LL_DMA_STREAM_7, (uint32_t)buffer_addr, LL_USART_DMA_GetRegAddr(USART1), LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
    LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_7, len);
    LL_DMA_EnableIT_TC(DMA2, LL_DMA_STREAM_7);
    LL_DMA_EnableIT_TE(DMA2, LL_DMA_STREAM_7);
}
/////////////////////////////////////////DEBUG/////////////////////////////////////////

/////////////////////////////////////////CRSF//////////////////////////////////////////
// CRSF UART RX BUFFER
#define CRSF_UART_BUFFER_SIZE 128
volatile uint8_t crsf_uart_buffer_1[CRSF_UART_BUFFER_SIZE];
volatile uint16_t crsf_uart_buffer_1_size = 0;
volatile uint8_t crsf_uart_buffer_2[CRSF_UART_BUFFER_SIZE];
volatile uint16_t crsf_uart_buffer_2_size = 0;
volatile uint8_t crsf_uart_buffer_index = 0;
void crsf_uart_callback(void)
{
    uint8_t data = LL_USART_ReceiveData8(UART4);
    if(crsf_uart_buffer_index == 0){
        if(crsf_uart_buffer_1_size >= CRSF_UART_BUFFER_SIZE){//full
            crsf_uart_buffer_2_size = 0;
            crsf_uart_buffer_index = 1;
            crsf_uart_buffer_2[crsf_uart_buffer_2_size] = data;
            crsf_uart_buffer_2_size ++;
        }else{
            crsf_uart_buffer_1[crsf_uart_buffer_1_size] = data;
            crsf_uart_buffer_1_size ++;
        }
    }else if(crsf_uart_buffer_index == 1){
        if(crsf_uart_buffer_2_size >= CRSF_UART_BUFFER_SIZE){//full
            crsf_uart_buffer_1_size = 0;
            crsf_uart_buffer_index = 0;
            crsf_uart_buffer_1[crsf_uart_buffer_1_size] = data;
            crsf_uart_buffer_1_size ++;
        }else{
            crsf_uart_buffer_2[crsf_uart_buffer_2_size] = data;
            crsf_uart_buffer_2_size ++;
        }
    }
}
// 高频率运行 处理UART缓冲区数据
void crsf_process_uart_buffer(void)
{
    if(crsf_uart_buffer_index == 1){ // 当前buffer1空闲 可以读取
        for(uint16_t i=0; i< crsf_uart_buffer_1_size; i++){
            crsf_process_byte(crsf_uart_buffer_1[i]);
        }
        crsf_uart_buffer_1_size = 0; // 复位
    }else if(crsf_uart_buffer_index == 0){ // 当前buffer2空闲 可以读取
        for(uint16_t i=0; i< crsf_uart_buffer_2_size; i++){
            crsf_process_byte(crsf_uart_buffer_2[i]);
        }
        crsf_uart_buffer_2_size = 0; // 复位
    }
}
void crsf_uart_start(void)
{
    LL_USART_ClearFlag_ORE(UART4);
    LL_USART_EnableIT_RXNE(UART4);
    // LL_USART_EnableIT_ERROR(UART4);
}
/////////////////////////////////////////CRSF//////////////////////////////////////////

////////////////////////////////////////SYSTICK////////////////////////////////////////
volatile bool system_1khz_update = false;
volatile uint32_t system_1khz_tick = 0;
void system_1khz_timer_start(void)
{
    LL_TIM_ClearFlag_UPDATE(TIM6);
    LL_TIM_EnableIT_UPDATE(TIM6);
    LL_TIM_EnableCounter(TIM6);
}
uint32_t get_system_1khz_tick(void)
{
    return system_1khz_tick;
}
void system_1khz_timer_callback(void)
{
    system_1khz_update = true;
    system_1khz_tick ++;
}
////////////////////////////////////////SYSTICK////////////////////////////////////////

////////////////////////////////////////BUTTON/////////////////////////////////////////
bool button1_pressed(void)
{
    return !HAL_GPIO_ReadPin(BUTTON1_GPIO_Port, BUTTON1_Pin);
}
bool button2_pressed(void)
{
    return !HAL_GPIO_ReadPin(BUTTON2_GPIO_Port, BUTTON2_Pin);
}
////////////////////////////////////////BUTTON/////////////////////////////////////////

////////////////////////////////////////SI24R1/////////////////////////////////////////
uint8_t si24r1_rx_buffer[32];
uint8_t si24r1_tx_buffer[32];
uint8_t si24r1_rx_len = 0;
uint32_t si24r1_rx_timestamp = 0;
////////////////////////////////////////SI24R1/////////////////////////////////////////

icm42688_data_t imu_data;
uint32_t last_tp_ms = 0;
uint16_t log_count = 0;
vfloat roll, pitch, yaw;     // 三轴欧拉角姿态
vfloat dcm[3][3];            // 当前旋转矩阵
vfloat dcm_goal[3][3];       // 目标旋转矩阵
vfloat omega_filtered[3];    // 滤波之后的角速度
vfloat acc_filtered[3];      // 滤波之后的加速度
vfloat omega_goal[3];        // 目标角速度
vfloat so3_output[3];        // SO3控制输出
vfloat rf_last[3];           // 上一次的RF杆量
uint32_t rf_last_timestamp = 0;

// ACC GYRO 低通滤波器(实际因为延迟较大 可不使用)
Butter3 omega_x_filter;
Butter3 omega_y_filter;
Butter3 omega_z_filter;
Butter3 acc_x_filter;
Butter3 acc_y_filter;
Butter3 acc_z_filter;
Butter3 rf_roll_filter;
Butter3 rf_pitch_filter;
Butter3 rf_yaw_filter;
// ACC GYRO 低通滤波器参数
vfloat acc_b[4] = {0.00289819, 0.00869458, 0.00869458, 0.00289819};
vfloat acc_a[4] = {1.0, -2.37409474, 1.92935567, -0.53207537};
// 100Hz cutoff
// vfloat gyro_b[4] = {0.01809893, 0.0542968, 0.0542968, 0.01809893};
// vfloat gyro_a[4] = {1.0, -1.76004188, 1.18289326, -0.27805992};
// 50Hz cutoff
vfloat gyro_b[4] = {0.00289819, 0.00869458, 0.00869458, 0.00289819};
vfloat gyro_a[4] = {1.0, -2.37409474, 1.92935567, -0.53207537};
// RF RPY 低通滤波器参数
vfloat rf_b[4] = {0.01809893, 0.0542968, 0.0542968, 0.01809893};
vfloat rf_a[4] = {1.0, -1.76004188, 1.18289326, -0.27805992};

void system_init(void)
{
    slog("System Init\r\n");

    slog("DSHOT Init\r\n");
    dshot_update_channel(0, 0, false);
    dshot_update_channel(1, 0, false);
    dshot_update_channel(2, 0, false);
    dshot_update_channel(3, 0, false);
    dshot_configure_dma_burst();

    if(!icm42688_init()){
        slog("ICM-42688-P Init Failed\r\n");
    }else{
        slog("ICM-42688-P Init Success\r\n");
    }

    slog("SI24R1 Init\r\n");
    si24r1_init(MODE_RX);

    slog("AHRS DCM Init\r\n");
    ahrs_dcm_init();

    slog("Start 1KHz Timer\r\n");
    system_1khz_timer_start();

    slog("CRSF Start\r\n");
    crsf_uart_start();

    // SO3控制器参数初始化
    so3_controller_param.gain[0] = -1.0;
    so3_controller_param.gain[1] = -1.0;
    so3_controller_param.gain[2] = -1.0;

    so3_controller_param.kr[0] = 1.3;
    so3_controller_param.kr[1] = 1.3;
    so3_controller_param.kr[2] = 0.5;

    so3_controller_param.kw[0] = 0.5;
    so3_controller_param.kw[1] = 0.5;
    so3_controller_param.kw[2] = 0.4;

    so3_controller_param.ki[0] = 0.0;
    so3_controller_param.ki[1] = 0.0;
    so3_controller_param.ki[2] = 0.0;

    slog("Butter3 Filter Init\r\n");
    butter3_filter_init(&omega_x_filter, gyro_b, gyro_a);
    butter3_filter_init(&omega_y_filter, gyro_b, gyro_a);
    butter3_filter_init(&omega_z_filter, gyro_b, gyro_a);
    butter3_filter_init(&acc_x_filter, acc_b, acc_a);
    butter3_filter_init(&acc_y_filter, acc_b, acc_a);
    butter3_filter_init(&acc_z_filter, acc_b, acc_a);
    butter3_filter_init(&rf_roll_filter, rf_b, rf_a);
    butter3_filter_init(&rf_pitch_filter, rf_b, rf_a);
    butter3_filter_init(&rf_yaw_filter, rf_b, rf_a);
}

uint8_t imu_log_buffer[32];
void system_loop(void)
{
    if(system_1khz_update){
        system_1khz_update = false;
        uint32_t tp_ms = get_system_1khz_tick();
        uint32_t dt_ms = tp_ms - last_tp_ms;
        // IMU更新
        icm42688_update(&imu_data);
        // 滤波器更新
        omega_filtered[0] = imu_data.gyro_rads[0];
        omega_filtered[1] = imu_data.gyro_rads[1];
        omega_filtered[2] = imu_data.gyro_rads[2];
        // GYRO滤波
        // omega_filtered[0] = butter3_filter_update(&omega_x_filter, imu_data.gyro_rads[0]);
        // omega_filtered[1] = butter3_filter_update(&omega_y_filter, imu_data.gyro_rads[1]);
        // omega_filtered[2] = butter3_filter_update(&omega_z_filter, imu_data.gyro_rads[2]);
        // ACC滤波
        acc_filtered[0] = butter3_filter_update(&acc_x_filter, imu_data.acc_ms2[0]);
        acc_filtered[1] = butter3_filter_update(&acc_y_filter, imu_data.acc_ms2[1]);
        acc_filtered[2] = butter3_filter_update(&acc_z_filter, imu_data.acc_ms2[2]);

        ahrs_dcm_update(omega_filtered, acc_filtered, NULL, (vfloat)dt_ms/1000.0f); // AHRS DCM更新
        ahrs_dcm_get_euler(&roll, &pitch, &yaw); // 更新欧拉角
        ahrs_dcm_get_dcm(dcm); // 更新旋转矩阵

        // 20250710 打包发送IMU数据 以及对应油门数据 用于滤波器debug
        // imu_log_buffer[0] = 0xAA;
        // memcpy(&imu_log_buffer[1], &tp_ms, 4);
        // memcpy(&imu_log_buffer[5], (void *)omega_filtered, 12);
        // memcpy(&imu_log_buffer[17], (void *)acc_filtered, 12);
        // memcpy(&imu_log_buffer[29], &crsf_data.throttle, 4);
        // debug_uart_tx_dma((uint32_t)imu_log_buffer, 33);

        // 设置欧拉角偏置 用于补偿机械偏角
        // ahrs_dcm_set_euler_bias(roll, pitch, yaw);
        // ahrs_dcm_set_dcm_bias(dcm);

        vfloat crsf_norm_scale = DEG2RAD(60.0f); // 归一化输入到目标欧拉角度的比例
        Matrix3FromEuler(crsf_norm_scale * crsf_data.roll, crsf_norm_scale * crsf_data.pitch, crsf_norm_scale * crsf_data.yaw, dcm_goal); // 使用接收机作为目标姿态
        // Vector3Zero(omega_goal); // 目标角速度初始化为0

        // 根据摇杆变化量计算目标角速度
        #define OMEGA_BOOST (10)
        if(crsf_data.timestamp_us > rf_last_timestamp){
            rf_last_timestamp = crsf_data.timestamp_us;
            omega_goal[0] = butter3_filter_update(&rf_roll_filter,  crsf_data.roll-rf_last[0] ) * crsf_norm_scale * OMEGA_BOOST;
            omega_goal[1] = butter3_filter_update(&rf_pitch_filter, crsf_data.pitch-rf_last[1]) * crsf_norm_scale * OMEGA_BOOST;
            omega_goal[2] = butter3_filter_update(&rf_yaw_filter,   crsf_data.yaw-rf_last[2]  ) * crsf_norm_scale * OMEGA_BOOST;
            rf_last[0] = crsf_data.roll;
            rf_last[1] = crsf_data.pitch;
            rf_last[2] = crsf_data.yaw;
            // slog_dma("omega goal: %.6f \t%.6f \t%.6f\r\n", omega_goal[0], omega_goal[1], omega_goal[2]);
        }

        // SO3 Control
        so3_controller_update(dcm, dcm_goal, omega_filtered, omega_goal, so3_output);
        // 控制量限幅
        vfloat roll_control  = so3_output[0];
        vfloat pitch_control = so3_output[1];
        vfloat yaw_control   = so3_output[2];
        roll_control  = (roll_control > 1.0f)? 1.0f : ((roll_control < -1.0f)? -1.0f : roll_control);
        pitch_control = (pitch_control > 1.0f)? 1.0f : ((pitch_control < -1.0f)? -1.0f : pitch_control);
        yaw_control   = (yaw_control > 1.0f)? 1.0f : ((yaw_control < -1.0f)? -1.0f : yaw_control);

        vfloat motor_speed[4]; // 电机速度数组
        // 直接使用遥控器控制(动力分配测试):
        // mixer_update(crsf_data.roll, crsf_data.pitch, crsf_data.yaw, crsf_data.throttle, motor_speed);

        // 使用SO3控制 电机动力分配:
        mixer_update(roll_control, pitch_control, yaw_control, crsf_data.throttle, motor_speed);

        // DSHOT Update
        // ARM SWITCH CH5
        if(crsf_data.button1){ // ARMED
            dshot_update_channel_norm(0, motor_speed[0], false);
            dshot_update_channel_norm(1, motor_speed[1], false);
            dshot_update_channel_norm(2, motor_speed[2], false);
            dshot_update_channel_norm(3, motor_speed[3], false);
        }else{ // DISARM
            dshot_disarm(0);
            dshot_disarm(1);
            dshot_disarm(2);
            dshot_disarm(3);
        }
        dshot_dma_burst_start();

        log_count++;
        if(log_count > 10){ // 100Hz UART DMA 1M Baudrate Debug Output
            log_count = 0;
            // slog_dma("rf last: %.6f \t%.6f \t%.6f\r\n", rf_last[0], rf_last[1], rf_last[2]);
            // slog_dma("sqrt: %.2f\r\n", simple_sqrt(2.0f));
            // slog_dma("speed: %.2f %.2f %.2f %.2f\r\n", motor_speed[0], motor_speed[1], motor_speed[2], motor_speed[3]);
            // slog_dma("omagef: %.4f %.4f %.4f\r\n", omega_filtered[0], omega_filtered[1], omega_filtered[2]);
            // slog_dma("kr: %.2f %.2f %.2f kw: %.2f %.2f %.2f\r\n", 
            //     so3_controller_param.kr[0], so3_controller_param.kr[1], so3_controller_param.kr[2], 
            //     so3_controller_param.kw[0], so3_controller_param.kw[1], so3_controller_param.kw[2]);
            // slog_dma("Control: roll: %.2f pitch: %.2f yaw: %.2f\r\n", roll_control, pitch_control, yaw_control);
            // slog_dma("Euler: %.4f %.4f %.4f\r\n", RAD2DEG(roll), RAD2DEG(pitch), RAD2DEG(yaw));
            // slog_dma("SO3: %.2f %.2f %.2f\r\n", so3_output[0], so3_output[1], so3_output[2]);
        }
        last_tp_ms = tp_ms;
    }else{
        crsf_process_uart_buffer(); // 处理CRSF接收机

        si24r1_rx_len = si24r1_check_rx(si24r1_rx_buffer); // 处理SI24R1接收机
        if(si24r1_rx_len == 32){
            si24r1_rx_timestamp = get_system_1khz_tick();
            // 调用处理函数
            bool ret = param_parse_frame(si24r1_rx_buffer);
            if(!ret){
                // slog("Param Parse Failed\r\n");
            }else{
                // slog("Param Parse Success\r\n");
            }
        }
    }
}
