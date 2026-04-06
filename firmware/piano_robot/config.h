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

constexpr int home_switch_pin = 2;
constexpr int far_limit_switch_pin = -1;   /* far-side safety limit switch (disabled) */
constexpr int user_button_pin = -1;        /* start/stop button (disabled)  */
constexpr bool home_switch_active_level = HIGH;
constexpr bool far_limit_active_level = HIGH;
constexpr bool user_button_active_level = LOW;
constexpr bool finger_active_level = HIGH;

constexpr size_t finger_count = 5U;
constexpr int finger_pins[finger_count] = {14, 12, 27, 26, 25};
constexpr uint8_t finger_pwm_channels[finger_count] = {2U, 3U, 4U, 5U, 6U};
constexpr uint32_t finger_pwm_frequency_hz = 1000U;
constexpr uint8_t finger_pwm_resolution_bits = 10U;
constexpr float finger_pwm_pressed_default_duty = 1.0f;

constexpr float encoder_cpr_motor = 64.0f;
constexpr float gear_ratio = 30.0f;
constexpr float encoder_cpr_output = encoder_cpr_motor * gear_ratio;
/* mm of carriage travel per one full output-shaft revolution.
 * = pulley_teeth * belt_pitch_mm
 * GT2 belt (2 mm pitch), 20-tooth pulley  ->  20 * 2 = 40 mm/rev  (VERIFY THIS)
 * If the robot still reads wrong, measure: move a known distance (e.g. 100 mm
 * with a ruler), read encoder_counts from telemetry, then:
 *   linear_travel_per_rev = (measured_mm / encoder_counts) * encoder_cpr_output
 * Previous value was 1.0f which caused ~37x under-reporting. */
constexpr float linear_travel_per_rev = 40.0f;
constexpr float position_to_angle_deg_scale = 360.0f / linear_travel_per_rev;
constexpr float angle_deg_to_position_scale = linear_travel_per_rev / 360.0f;
}

#endif
