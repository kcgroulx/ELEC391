#include "platform_io.h"

#include "config.h"
#include "esp32-hal-ledc.h"

#if __has_include("driver/pulse_cnt.h")
#include "driver/pulse_cnt.h"
#define APP_USE_PCNT_V2 1
#else
#include "driver/pcnt.h"
#define APP_USE_PCNT_V2 0
#endif

namespace
{
constexpr uint32_t encoder_glitch_filter_ns = 5000U;
constexpr uint16_t encoder_glitch_filter_apb_cycles = 400U;
constexpr int encoder_count_high_limit = 30000;
constexpr int encoder_count_low_limit = -30000;

#if APP_USE_PCNT_V2
pcnt_unit_handle_t encoderUnit = nullptr;
pcnt_channel_handle_t encoderChannelA = nullptr;
pcnt_channel_handle_t encoderChannelB = nullptr;
#else
constexpr pcnt_unit_t encoderUnit = PCNT_UNIT_0;
#endif

uint8_t currentFingerBitmask = 0U;
float fingerPressedDuty = app_config::finger_pwm_pressed_default_duty;

uint32_t pwm_max_duty(uint8_t resolutionBits)
{
    return (1UL << resolutionBits) - 1UL;
}

void write_pwm_pin(uint8_t pin, float duty, bool activeLow, uint8_t resolutionBits)
{
    const float clampedDuty = constrain(duty, 0.0f, 1.0f);
    const float physicalDuty = activeLow ? (1.0f - clampedDuty) : clampedDuty;
    const uint32_t dutyCounts = static_cast<uint32_t>(physicalDuty * static_cast<float>(pwm_max_duty(resolutionBits)));
    ledcWrite(pin, dutyCounts);
}

#if APP_USE_PCNT_V2
void configure_encoder_pcnt(void)
{
    pcnt_unit_config_t unitConfig = {};
    unitConfig.low_limit = encoder_count_low_limit;
    unitConfig.high_limit = encoder_count_high_limit;
    unitConfig.intr_priority = 0;
    unitConfig.flags.accum_count = 1;
    pcnt_new_unit(&unitConfig, &encoderUnit);

    pcnt_glitch_filter_config_t filterConfig = {};
    filterConfig.max_glitch_ns = encoder_glitch_filter_ns;
    pcnt_unit_set_glitch_filter(encoderUnit, &filterConfig);

    pcnt_chan_config_t channelAConfig = {};
    channelAConfig.edge_gpio_num = app_config::encoder_a_pin;
    channelAConfig.level_gpio_num = app_config::encoder_b_pin;
    pcnt_new_channel(encoderUnit, &channelAConfig, &encoderChannelA);
    pcnt_channel_set_edge_action(
        encoderChannelA,
        PCNT_CHANNEL_EDGE_ACTION_INCREASE,
        PCNT_CHANNEL_EDGE_ACTION_DECREASE);
    pcnt_channel_set_level_action(
        encoderChannelA,
        PCNT_CHANNEL_LEVEL_ACTION_INVERSE,
        PCNT_CHANNEL_LEVEL_ACTION_KEEP);

    pcnt_chan_config_t channelBConfig = {};
    channelBConfig.edge_gpio_num = app_config::encoder_b_pin;
    channelBConfig.level_gpio_num = app_config::encoder_a_pin;
    pcnt_new_channel(encoderUnit, &channelBConfig, &encoderChannelB);
    pcnt_channel_set_edge_action(
        encoderChannelB,
        PCNT_CHANNEL_EDGE_ACTION_DECREASE,
        PCNT_CHANNEL_EDGE_ACTION_INCREASE);
    pcnt_channel_set_level_action(
        encoderChannelB,
        PCNT_CHANNEL_LEVEL_ACTION_INVERSE,
        PCNT_CHANNEL_LEVEL_ACTION_KEEP);

    pcnt_unit_enable(encoderUnit);
    pcnt_unit_clear_count(encoderUnit);
    pcnt_unit_start(encoderUnit);
}
#else
void configure_encoder_pcnt(void)
{
    pcnt_config_t channelAConfig = {};
    channelAConfig.pulse_gpio_num = app_config::encoder_a_pin;
    channelAConfig.ctrl_gpio_num = app_config::encoder_b_pin;
    channelAConfig.lctrl_mode = PCNT_MODE_KEEP;
    channelAConfig.hctrl_mode = PCNT_MODE_REVERSE;
    channelAConfig.pos_mode = PCNT_COUNT_INC;
    channelAConfig.neg_mode = PCNT_COUNT_DEC;
    channelAConfig.counter_h_lim = encoder_count_high_limit;
    channelAConfig.counter_l_lim = encoder_count_low_limit;
    channelAConfig.unit = encoderUnit;
    channelAConfig.channel = PCNT_CHANNEL_0;
    pcnt_unit_config(&channelAConfig);

    pcnt_config_t channelBConfig = {};
    channelBConfig.pulse_gpio_num = app_config::encoder_b_pin;
    channelBConfig.ctrl_gpio_num = app_config::encoder_a_pin;
    channelBConfig.lctrl_mode = PCNT_MODE_KEEP;
    channelBConfig.hctrl_mode = PCNT_MODE_REVERSE;
    channelBConfig.pos_mode = PCNT_COUNT_DEC;
    channelBConfig.neg_mode = PCNT_COUNT_INC;
    channelBConfig.counter_h_lim = encoder_count_high_limit;
    channelBConfig.counter_l_lim = encoder_count_low_limit;
    channelBConfig.unit = encoderUnit;
    channelBConfig.channel = PCNT_CHANNEL_1;
    pcnt_unit_config(&channelBConfig);

    pcnt_set_filter_value(encoderUnit, encoder_glitch_filter_apb_cycles);
    pcnt_filter_enable(encoderUnit);
    pcnt_counter_pause(encoderUnit);
    pcnt_counter_clear(encoderUnit);
    pcnt_counter_resume(encoderUnit);
}
#endif
}

void platform_io_init(void)
{
    pinMode(app_config::encoder_a_pin, INPUT_PULLUP);
    pinMode(app_config::encoder_b_pin, INPUT_PULLUP);
    if (app_config::home_switch_pin >= 0)
    {
        pinMode(app_config::home_switch_pin, INPUT_PULLUP);
    }
    if (app_config::far_limit_switch_pin >= 0)
    {
        pinMode(app_config::far_limit_switch_pin, INPUT_PULLUP);
    }
    if (app_config::user_button_pin >= 0)
    {
        pinMode(app_config::user_button_pin, INPUT_PULLUP);
    }

    for (size_t i = 0; i < app_config::finger_count; ++i)
    {
        ledcAttachChannel(
            app_config::finger_pins[i],
            app_config::finger_pwm_frequency_hz,
            app_config::finger_pwm_resolution_bits,
            app_config::finger_pwm_channels[i]);
        write_pwm_pin(
            app_config::finger_pins[i],
            0.0f,
            !app_config::finger_active_level,
            app_config::finger_pwm_resolution_bits);
    }

    ledcAttachChannel(
        app_config::motor_pwm_forward_pin,
        app_config::motor_pwm_frequency_hz,
        app_config::motor_pwm_resolution_bits,
        app_config::motor_pwm_forward_channel);
    ledcAttachChannel(
        app_config::motor_pwm_reverse_pin,
        app_config::motor_pwm_frequency_hz,
        app_config::motor_pwm_resolution_bits,
        app_config::motor_pwm_reverse_channel);
    configure_encoder_pcnt();

    platform_io_set_motor_pwm(0.0f, 0.0f);
    platform_io_set_fingers(0x00);
}

void platform_io_set_motor_pwm(float forwardDuty, float reverseDuty)
{
    write_pwm_pin(
        app_config::motor_pwm_forward_pin,
        forwardDuty,
        app_config::motor_pwm_active_low,
        app_config::motor_pwm_resolution_bits);
    write_pwm_pin(
        app_config::motor_pwm_reverse_pin,
        reverseDuty,
        app_config::motor_pwm_active_low,
        app_config::motor_pwm_resolution_bits);
}

void platform_io_set_finger_pressed_duty(float duty)
{
    fingerPressedDuty = constrain(duty, 0.0f, 1.0f);
    platform_io_set_fingers(currentFingerBitmask);
}

float platform_io_get_finger_pressed_duty(void)
{
    return fingerPressedDuty;
}

int32_t platform_io_get_encoder_count(void)
{
#if APP_USE_PCNT_V2
    int count = 0;
    pcnt_unit_get_count(encoderUnit, &count);
    return count;
#else
    int16_t count = 0;
    pcnt_get_counter_value(encoderUnit, &count);
    return count;
#endif
}

uint32_t platform_io_get_encoder_isr_count(void)
{
    return 0U;
}

bool platform_io_get_encoder_a_state(void)
{
    return digitalRead(app_config::encoder_a_pin) == HIGH;
}

bool platform_io_get_encoder_b_state(void)
{
    return digitalRead(app_config::encoder_b_pin) == HIGH;
}

void platform_io_zero_encoder(void)
{
#if APP_USE_PCNT_V2
    pcnt_unit_clear_count(encoderUnit);
#else
    pcnt_counter_pause(encoderUnit);
    pcnt_counter_clear(encoderUnit);
    pcnt_counter_resume(encoderUnit);
#endif
}

bool platform_io_is_home_switch_active(void)
{
    if (app_config::home_switch_pin < 0)
    {
        return false;
    }
    return digitalRead(app_config::home_switch_pin) == app_config::home_switch_active_level;
}

bool platform_io_is_far_limit_active(void)
{
    if (app_config::far_limit_switch_pin < 0)
    {
        return false;
    }
    return digitalRead(app_config::far_limit_switch_pin) == app_config::far_limit_active_level;
}

bool platform_io_is_user_button_active(void)
{
    if (app_config::user_button_pin < 0)
    {
        return false;
    }
    return digitalRead(app_config::user_button_pin) == app_config::user_button_active_level;
}

void platform_io_set_fingers(uint8_t fingerBitmask)
{
    currentFingerBitmask = fingerBitmask;
    for (size_t i = 0; i < app_config::finger_count; ++i)
    {
        const bool active = (fingerBitmask & (1U << i)) != 0U;
        write_pwm_pin(
            app_config::finger_pins[i],
            active ? fingerPressedDuty : 0.0f,
            !app_config::finger_active_level,
            app_config::finger_pwm_resolution_bits);
    }
}

uint32_t platform_io_millis(void)
{
    return millis();
}
