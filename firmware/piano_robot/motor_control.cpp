#include "motor_control.h"

#include <Arduino.h>
#include <stdint.h>

#include "config.h"
#include "platform_io.h"

namespace
{
int32_t encoderRawLastCount = 0;
int32_t encoderPositionCounts = 0;
}

void motor_control_init(void)
{
    platform_io_init();
    encoderRawLastCount = platform_io_get_encoder_count();
    encoderPositionCounts = 0;
    motor_control_set_motor_speed(0.0f);
}

void motor_control_set_motor_speed(float speed)
{
    const float clampedSpeed = constrain(-speed, -1.0f, 1.0f);

    if (clampedSpeed > 0.0f)
    {
        platform_io_set_motor_pwm(clampedSpeed, 0.0f);
    }
    else if (clampedSpeed < 0.0f)
    {
        platform_io_set_motor_pwm(0.0f, -clampedSpeed);
    }
    else
    {
        platform_io_set_motor_pwm(0.0f, 0.0f);
    }
}

void motor_control_update_encoder(void)
{
    const int32_t encoderCurrentCount = platform_io_get_encoder_count();
    const int32_t encoderDeltaCount = encoderCurrentCount - encoderRawLastCount;

    encoderRawLastCount = encoderCurrentCount;
    encoderPositionCounts -= encoderDeltaCount;
}

int32_t motor_control_get_position_counts(void)
{
    return encoderPositionCounts;
}

float motor_control_get_angle_deg(void)
{
    return (static_cast<float>(encoderPositionCounts) / app_config::encoder_cpr_output) * 360.0f;
}

float motor_control_get_linear_position(void)
{
    return motor_control_get_angle_deg() * app_config::angle_deg_to_position_scale;
}

void motor_control_zero_position(void)
{
    platform_io_zero_encoder();
    encoderRawLastCount = 0;
    encoderPositionCounts = 0;
}
