#pragma once

#include "main.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stddef.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include "usart.h"

void slog(const char *fmt, ...);
void slog_dma(const char *fmt, ...);
void debug_uart_tx(uint8_t *buffer, uint32_t len);
void debug_uart_tx_dma_cplt_callback(void);
void debug_uart_tx_dma_enable(void);
void debug_uart_tx_dma_disable(void);
void debug_uart_configure_tx_dma(uint32_t buffer_addr, uint32_t len);
void debug_uart_tx_dma(uint32_t buffer_addr, uint32_t len);

void system_init(void);
void system_loop(void);

void crsf_uart_callback(void);
void crsf_process_uart_buffer(void);
void crsf_uart_start(void);

void system_1khz_timer_start(void);
void system_1khz_timer_callback(void);
uint32_t get_system_1khz_tick(void);

bool button1_pressed(void);
bool button2_pressed(void);
