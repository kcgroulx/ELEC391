#include "config.h"
#include "hal_interface.h"
#include "motor_control.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

hw_timer_t* g_pidTimer = NULL;

namespace
{
constexpr uint32_t kStatusIntervalMs = 100U;
constexpr size_t kCommandBufferSize = 64U;
constexpr float kTargetMinMm = 0.0f;
constexpr float kTargetMaxMm = 350.0f;
constexpr float kMotorSpeedMin = -1.0f;
constexpr float kMotorSpeedMax = 1.0f;

char s_commandBuffer[kCommandBufferSize];
size_t s_commandLength = 0U;
float s_targetMm = 0.0f;
float s_manualMotorSpeed = 0.0f;
uint8_t s_fingerMask = 0U;
uint32_t s_lastStatusMs = 0U;
bool s_manualMotorMode = false;
}

void IRAM_ATTR onPIDTimer(void)
{
    hal_flagPIDPending();
}

float mm_to_deg(float positionMm)
{
    return positionMm * app_config::position_to_angle_deg_scale;
}

float control_mm_to_ui_mm(float positionMm)
{
    return -positionMm;
}

float ui_mm_to_control_mm(float positionMm)
{
    return -positionMm;
}

float ui_speed_to_control_speed(float speed)
{
    return -speed;
}

float clamp_target_mm(float positionMm)
{
    if (positionMm < kTargetMinMm)
    {
        return kTargetMinMm;
    }

    if (positionMm > kTargetMaxMm)
    {
        return kTargetMaxMm;
    }

    return positionMm;
}

float clamp_motor_speed(float speed)
{
    if (speed < kMotorSpeedMin)
    {
        return kMotorSpeedMin;
    }

    if (speed > kMotorSpeedMax)
    {
        return kMotorSpeedMax;
    }

    return speed;
}

void sync_pid_target_to_current_position(void)
{
    const float currentControlMm = motor_control_get_linear_position();
    s_targetMm = control_mm_to_ui_mm(currentControlMm);
    hal_motorSetTarget(currentControlMm);
    hal_motorNotifyArrived();
}

void print_status(void)
{
    const float actualMm = control_mm_to_ui_mm(motor_control_get_linear_position());

    Serial.print("STATUS mode=");
    Serial.print(s_manualMotorMode ? "MANUAL" : "PID");
    Serial.print(" manual_speed=");
    Serial.print(s_manualMotorSpeed, 2);
    Serial.print(" target_mm=");
    Serial.print(s_targetMm, 2);
    Serial.print(" actual_mm=");
    Serial.print(actualMm, 2);
    Serial.print(" actual_deg=");
    Serial.print(mm_to_deg(actualMm), 1);
    Serial.print(" encoder=");
    Serial.print(motor_control_get_position_counts());
    Serial.print(" arrived=");
    Serial.print(s_manualMotorMode ? 0 : hal_motorHasArrived());
    Serial.print(" fingers=0x");
    if (s_fingerMask < 0x10U)
    {
        Serial.print('0');
    }
    Serial.println(s_fingerMask, HEX);
}

void set_target_mm(float targetMm)
{
    const float clampedTargetMm = clamp_target_mm(targetMm);
    s_manualMotorMode = false;
    s_manualMotorSpeed = 0.0f;
    motor_control_set_motor_speed(0.0f);
    s_targetMm = clampedTargetMm;
    hal_motorSetTarget(ui_mm_to_control_mm(clampedTargetMm));
    Serial.print("ACK target_mm=");
    Serial.print(s_targetMm, 2);
    Serial.print(" target_deg=");
    Serial.println(mm_to_deg(s_targetMm), 1);
}

void set_manual_motor_speed(float speed)
{
    const float clampedSpeed = clamp_motor_speed(speed);

    if (clampedSpeed == 0.0f)
    {
        motor_control_set_motor_speed(0.0f);
        s_manualMotorMode = false;
        s_manualMotorSpeed = 0.0f;
        sync_pid_target_to_current_position();
    }
    else
    {
        s_manualMotorMode = true;
        s_manualMotorSpeed = clampedSpeed;
        motor_control_set_motor_speed(ui_speed_to_control_speed(clampedSpeed));
    }

    Serial.print("ACK motor_speed=");
    Serial.println(s_manualMotorSpeed, 2);
}

void zero_position(void)
{
    motor_control_set_motor_speed(0.0f);
    motor_control_zero_position();
    s_manualMotorMode = false;
    s_manualMotorSpeed = 0.0f;
    s_targetMm = 0.0f;
    hal_motorSetTarget(0.0f);
    hal_motorNotifyArrived();
    Serial.println("ACK zero=1");
}

void set_solenoid(uint8_t fingerIndex, bool active)
{
    if (fingerIndex >= app_config::finger_count)
    {
        return;
    }

    if (active)
    {
        hal_fingerPress(fingerIndex);
        s_fingerMask |= static_cast<uint8_t>(1U << fingerIndex);
    }
    else
    {
        hal_fingerRelease(fingerIndex);
        s_fingerMask &= static_cast<uint8_t>(~(1U << fingerIndex));
    }

    Serial.print("ACK solenoid=");
    Serial.print(fingerIndex + 1U);
    Serial.print(" state=");
    Serial.println(active ? 1 : 0);
}

void all_solenoids_off(void)
{
    hal_fingerReleaseAll();
    s_fingerMask = 0U;
    Serial.println("ACK all_solenoids=0");
}

void handle_command(char* command)
{
    while (*command == ' ')
    {
        ++command;
    }

    if (*command == '\0')
    {
        return;
    }

    if (strncmp(command, "TARGET_MM ", 10) == 0)
    {
        set_target_mm(static_cast<float>(atof(command + 10)));
        return;
    }

    if (strncmp(command, "MOTOR ", 6) == 0)
    {
        set_manual_motor_speed(static_cast<float>(atof(command + 6)));
        return;
    }

    if (strcmp(command, "STATUS?") == 0)
    {
        print_status();
        return;
    }

    if (strcmp(command, "ZERO") == 0)
    {
        zero_position();
        return;
    }

    if (strcmp(command, "ALL_OFF") == 0)
    {
        all_solenoids_off();
        return;
    }

    unsigned int fingerNumber = 0U;
    unsigned int active = 0U;
    if (sscanf(command, "SOL %u %u", &fingerNumber, &active) == 2)
    {
        if ((fingerNumber >= 1U) && (fingerNumber <= app_config::finger_count) && (active <= 1U))
        {
            set_solenoid(static_cast<uint8_t>(fingerNumber - 1U), active == 1U);
        }
        else
        {
            Serial.println("ERR invalid_solenoid");
        }
        return;
    }

    Serial.println("ERR unknown_command");
}

void process_serial(void)
{
    while (Serial.available() > 0)
    {
        const char c = static_cast<char>(Serial.read());
        if ((c == '\r') || (c == '\n'))
        {
            s_commandBuffer[s_commandLength] = '\0';
            handle_command(s_commandBuffer);
            s_commandLength = 0U;
            continue;
        }

        if (s_commandLength + 1U < kCommandBufferSize)
        {
            s_commandBuffer[s_commandLength++] = c;
        }
        else
        {
            s_commandLength = 0U;
            Serial.println("ERR command_too_long");
        }
    }
}

void setup(void)
{
    Serial.begin(115200);

    motor_control_init();
    piano_hal_init();
    motor_control_zero_position();

    g_pidTimer = timerBegin(1000000);
    timerAttachInterrupt(g_pidTimer, &onPIDTimer);
    timerAlarm(g_pidTimer, app_config::control_period_us, true, 0);

    Serial.println("READY");
    print_status();
}

void loop(void)
{
    if (s_manualMotorMode)
    {
        motor_control_update_encoder();
    }
    else
    {
        hal_runPendingPID();
    }
    process_serial();

    const uint32_t now = millis();
    if ((now - s_lastStatusMs) >= kStatusIntervalMs)
    {
        s_lastStatusMs = now;
        print_status();
    }
}
