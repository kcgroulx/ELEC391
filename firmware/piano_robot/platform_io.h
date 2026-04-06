#ifndef PIANOBOT_PLATFORM_IO_H
#define PIANOBOT_PLATFORM_IO_H

#include <Arduino.h>
#include <stdint.h>

void platform_io_init(void);
void platform_io_set_motor_pwm(float forwardDuty, float reverseDuty);
int32_t platform_io_get_encoder_count(void);
uint32_t platform_io_get_encoder_isr_count(void);
bool platform_io_get_encoder_a_state(void);
bool platform_io_get_encoder_b_state(void);
void platform_io_zero_encoder(void);
bool platform_io_is_home_switch_active(void);
bool platform_io_is_far_limit_active(void);
bool platform_io_is_user_button_active(void);
void platform_io_set_fingers(uint8_t fingerBitmask);
uint32_t platform_io_millis(void);

#endif
