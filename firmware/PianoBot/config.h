#ifndef PIANOBOT_CONFIG_H
#define PIANOBOT_CONFIG_H

#include <Arduino.h>
#include <stdint.h>

namespace app_config
{
constexpr uint32_t control_period_us = 1000U;

constexpr int motor_pwm_forward_pin = 32;
constexpr int motor_pwm_reverse_pin = 33;
constexpr uint8_t motor_pwm_forward_channel = 0U;
constexpr uint8_t motor_pwm_reverse_channel = 1U;
constexpr uint32_t motor_pwm_frequency_hz = 100U;
constexpr uint8_t motor_pwm_resolution_bits = 10U;
constexpr bool motor_pwm_active_low = true;

constexpr int encoder_a_pin = 18;
constexpr int encoder_b_pin = 19;

constexpr int home_switch_pin = -1;
constexpr int user_button_pin = -1;
constexpr bool home_switch_active_level = LOW;
constexpr bool user_button_active_level = LOW;
constexpr bool finger_active_level = HIGH;

constexpr size_t finger_count = 6U;
constexpr int finger_pins[finger_count] = {21, 22, 23, 25, 26, 27};

constexpr float encoder_cpr_motor = 64.0f;
constexpr float gear_ratio = 30.0f;
constexpr float encoder_cpr_output = encoder_cpr_motor * gear_ratio;
constexpr float linear_travel_per_rev = 1.0f;
constexpr float position_to_angle_deg_scale = 360.0f / linear_travel_per_rev;
constexpr float angle_deg_to_position_scale = linear_travel_per_rev / 360.0f;
}

#endif
