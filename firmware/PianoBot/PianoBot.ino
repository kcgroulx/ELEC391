#include <Arduino.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "config.h"
#include "motor_control.h"
#include "pid.h"
#include "platform_io.h"

namespace
{
constexpr size_t serialBufferSize = 96U;
constexpr uint32_t telemetryPeriodMs = 100U;

char serialBuffer[serialBufferSize] = {};
size_t serialBufferLength = 0U;
uint32_t lastControlTickUs = 0U;
uint32_t lastTelemetryTickMs = 0U;
uint32_t telemetrySequence = 0U;
float targetAngleDeg = 0.0f;
float lastMotorCommand = 0.0f;

float target_angle_deg_to_position(float angleDeg)
{
    return angleDeg * app_config::angle_deg_to_position_scale;
}

void print_status(void)
{
    Serial.print("STATUS seq=");
    Serial.print(telemetrySequence++);
    Serial.print(" t_ms=");
    Serial.print(millis());
    Serial.print(" kp=");
    Serial.print(pid_get_kp(), 6);
    Serial.print(" ki=");
    Serial.print(pid_get_ki(), 6);
    Serial.print(" kd=");
    Serial.print(pid_get_kd(), 6);
    Serial.print(" deadband_deg=");
    Serial.print(pid_get_deadband_deg(), 3);
    Serial.print(" settle_vel_deg_per_s=");
    Serial.print(pid_get_settle_velocity_deg_per_sec(), 3);
    Serial.print(" target_deg=");
    Serial.print(targetAngleDeg, 3);
    Serial.print(" angle_deg=");
    Serial.print(motor_control_get_angle_deg(), 3);
    Serial.print(" error_deg=");
    Serial.print(pid_get_last_error_deg(), 3);
    Serial.print(" output=");
    Serial.print(lastMotorCommand, 4);
    Serial.print(" counts=");
    Serial.println(motor_control_get_position_counts());
}

void reset_controller_state(void)
{
    pid_reset();
    lastMotorCommand = 0.0f;
    motor_control_set_motor_speed(0.0f);
}

void handle_zero(void)
{
    motor_control_zero_position();
    targetAngleDeg = 0.0f;
    reset_controller_state();
    Serial.println("OK ZERO");
    print_status();
}

bool parse_single_float(const char* text, float* value)
{
    char* endPtr = nullptr;
    const float parsedValue = strtof(text, &endPtr);
    if ((endPtr == text) || (*endPtr != '\0'))
    {
        return false;
    }

    *value = parsedValue;
    return true;
}

void handle_command(char* line)
{
    while ((*line == ' ') || (*line == '\t'))
    {
        ++line;
    }

    for (char* tail = line + strlen(line); tail > line; --tail)
    {
        if ((tail[-1] == ' ') || (tail[-1] == '\t'))
        {
            tail[-1] = '\0';
        }
        else
        {
            break;
        }
    }

    if (*line == '\0')
    {
        return;
    }

    float kp = 0.0f;
    float ki = 0.0f;
    float kd = 0.0f;
    float requestedDeadbandDeg = 0.0f;
    float requestedSettleVelocityDegPerSec = 0.0f;
    float requestedAngleDeg = 0.0f;

    if (sscanf(line, "PID %f %f %f", &kp, &ki, &kd) == 3)
    {
        pid_set_gains(kp, ki, kd);
        reset_controller_state();
        Serial.println("OK PID");
        print_status();
    }
    else if ((strncmp(line, "DEADBAND ", 9) == 0) && parse_single_float(line + 9, &requestedDeadbandDeg))
    {
        pid_set_deadband_deg(requestedDeadbandDeg);
        reset_controller_state();
        Serial.println("OK DEADBAND");
        print_status();
    }
    else if ((strncmp(line, "SETTLEVEL ", 10) == 0) && parse_single_float(line + 10, &requestedSettleVelocityDegPerSec))
    {
        pid_set_settle_velocity_deg_per_sec(requestedSettleVelocityDegPerSec);
        reset_controller_state();
        Serial.println("OK SETTLEVEL");
        print_status();
    }
    else if ((strncmp(line, "TARGET ", 7) == 0) && parse_single_float(line + 7, &requestedAngleDeg))
    {
        targetAngleDeg = requestedAngleDeg;
        reset_controller_state();
        Serial.println("OK TARGET");
        print_status();
    }
    else if (strcmp(line, "ZERO") == 0)
    {
        handle_zero();
    }
    else
    {
        Serial.print("ERR unknown_command ");
        Serial.println(line);
    }
}

void handle_serial_input(void)
{
    while (Serial.available() > 0)
    {
        const char incomingChar = static_cast<char>(Serial.read());

        if (incomingChar == '\r')
        {
            continue;
        }

        if (incomingChar == '\n')
        {
            serialBuffer[serialBufferLength] = '\0';
            handle_command(serialBuffer);
            serialBufferLength = 0U;
            continue;
        }

        if (serialBufferLength < (serialBufferSize - 1U))
        {
            serialBuffer[serialBufferLength++] = incomingChar;
        }
        else
        {
            serialBufferLength = 0U;
            Serial.println("ERR line_too_long");
        }
    }
}

void run_pid_control(void)
{
    const uint32_t nowUs = micros();
    if ((nowUs - lastControlTickUs) >= app_config::control_period_us)
    {
        lastControlTickUs = nowUs;
        motor_control_update_encoder();
        lastMotorCommand = pid_step(target_angle_deg_to_position(targetAngleDeg));
        motor_control_set_motor_speed(lastMotorCommand);
    }
}

void emit_periodic_telemetry(void)
{
    const uint32_t nowMs = millis();
    if ((nowMs - lastTelemetryTickMs) >= telemetryPeriodMs)
    {
        lastTelemetryTickMs = nowMs;
        print_status();
    }
}
}

void setup()
{
    Serial.begin(115200);
    motor_control_init();
    platform_io_set_fingers(0U);
    motor_control_zero_position();
    reset_controller_state();

    lastControlTickUs = micros();
    lastTelemetryTickMs = millis();

    Serial.println("READY PID_TEST");
    print_status();
}

void loop()
{
    handle_serial_input();
    run_pid_control();
    emit_periodic_telemetry();
}
