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
void system_init(void);
void system_loop(void);
void uart_buffer_reset(void);
void uart_rx_callback(uint8_t data);
