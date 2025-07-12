#include "system.h"
#include "si24r1.h"
#include "usart.h"

uint8_t uart_byte_buffer[1];
uint8_t uart_buffer[32];
uint8_t frame_buffer[32];
bool has_received_frame = false;
uint8_t uart_buffer_len = 0;
uint8_t uart_buffer_checksum = 0;
bool has_detected_frame_head = false;

#define LOG_UART_HANDLE huart1
uint8_t log_buffer[128];
void slog(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    vsnprintf((char*)log_buffer, sizeof(log_buffer), fmt, args);
    va_end(args);
    HAL_UART_Transmit(&LOG_UART_HANDLE, (uint8_t*)log_buffer, strlen((char*)log_buffer), 1000);
}

void uart_buffer_reset(void)
{
    has_detected_frame_head = false;
    uart_buffer_len = 0;
    uart_buffer_checksum = 0;
}

void uart_rx_callback(uint8_t data)
{
    if(data == 0xAA){
        has_detected_frame_head = true;
        return;
    }
    if(has_detected_frame_head){
        uart_buffer[uart_buffer_len] = data;
        uart_buffer_len++;
        if(uart_buffer_len >= 32){
            uint8_t check_sum = 0;
            for(uint8_t i=0; i<31; i++){
                check_sum += uart_buffer[i];
            }
            if(check_sum == uart_buffer[31]){
                for(uint8_t i=0; i<32; i++){
                    frame_buffer[i] = uart_buffer[i]; // Copy
                }
                has_received_frame = true;
            }
            uart_buffer_reset();
        }
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart == &LOG_UART_HANDLE){
        uart_rx_callback(uart_byte_buffer[0]);
        HAL_UART_Receive_IT(&LOG_UART_HANDLE, uart_byte_buffer, 1);
    }
}

// SI24R1 转发UART帧格式:
// 帧头: 0xAA
// 数据部分一共32字节 其中最后一个字节为校验和

void system_init(void)
{
    HAL_Delay(100);
    slog("System Init\r\n");
    slog("SI24R1 Init\r\n");
    si24r1_init(MODE_TX);
    slog("UART RX Start\r\n");
    HAL_UART_Receive_IT(&LOG_UART_HANDLE, uart_byte_buffer, 1);
}

void system_loop(void)
{
    if(has_received_frame){
        has_received_frame = false;
        // 转发至SI24R1 TX
        si24r1_fast_tx(frame_buffer);
        slog("SI24R1 Send Frame Success\r\n");
    }
}
