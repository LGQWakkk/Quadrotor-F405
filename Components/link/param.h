#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "system.h"

#include "so3_controller.h"
#include "ahrs_dcm.h"

#define FRAME_SERVO_DEBUG 0x10
#define FRAME_SERVO_MIN   0x11
#define FRAME_SERVO_MID   0x12
#define FRAME_SERVO_MAX   0x13

#define FRAME_SO3_PARAM   0x20

bool param_parse_frame(uint8_t *buf);
bool param_check_sum(uint8_t *buf, uint8_t ref_sum);
bool param_parse_servo_debug(uint8_t *buf);
bool param_parse_servo_min(uint8_t *buf);
bool param_parse_servo_max(uint8_t *buf);
bool param_parse_servo_mid(uint8_t *buf);
bool param_parse_so3_param(uint8_t *buf);
