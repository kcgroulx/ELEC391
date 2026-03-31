#include "pid.h"

#include <math.h>

#include "config.h"
#include "motor_control.h"

namespace
{
constexpr float dt = static_cast<float>(app_config::control_period_us) / 1000000.0f;

float kp = 0.0310f;
float ki = 0.0008f;
float kd = 0.002f;

float outputLimit = 1.0f;
float deadbandDeg = 2.0f;
float settleVelocityDegPerSec = 8.0f;
float nearTargetBandDeg = 12.0f;
float nearTargetOutputLimit = 0.25f;
float breakawayErrorDeg = 4.0f;
float iZoneDeg = 40.0f;
float integratorLimit = 200.0f;
float maxCommandStep = 0.02f;
float minEffectiveCommand = 0.08f;
float derivativeLpfAlpha = 0.15f;

float integrator = 0.0f;
float prevAngle = 0.0f;
float derivativeFiltered = 0.0f;
float prevCommand = 0.0f;
float lastTargetAngleDeg = 0.0f;
float lastAngleDeg = 0.0f;
float lastErrorDeg = 0.0f;

float clampf(float value, float low, float high)
{
    if (value < low)
    {
        return low;
    }

    if (value > high)
    {
        return high;
    }

    return value;
}
}

float pid_step(float targetPosition)
{
    const float targetAngleDeg = targetPosition * app_config::position_to_angle_deg_scale;
    const float angleDeg = motor_control_get_angle_deg();
    const float errorDeg = targetAngleDeg - angleDeg;
    const float absErrorDeg = fabsf(errorDeg);

    lastTargetAngleDeg = targetAngleDeg;
    lastAngleDeg = angleDeg;
    lastErrorDeg = errorDeg;

    const float angleRateDegPerSec = (angleDeg - prevAngle) / dt;
    prevAngle = angleDeg;
    derivativeFiltered += derivativeLpfAlpha * (angleRateDegPerSec - derivativeFiltered);
    const float absVelocityDegPerSec = fabsf(derivativeFiltered);

    if ((absErrorDeg <= deadbandDeg) && (absVelocityDegPerSec <= settleVelocityDegPerSec))
    {
        integrator = 0.0f;
        prevCommand = 0.0f;
        return 0.0f;
    }

    if (absErrorDeg <= iZoneDeg)
    {
        integrator += errorDeg * dt;
        integrator = clampf(integrator, -integratorLimit, integratorLimit);
    }
    else
    {
        integrator = 0.0f;
    }

    const float pTerm = kp * errorDeg;
    const float iTerm = ki * integrator;
    const float dTerm = -kd * derivativeFiltered;

    float command = pTerm + iTerm + dTerm;

    // Near the target, remove integral drive and clamp the command harder so the
    // motor settles instead of hunting back and forth around the setpoint.
    if (absErrorDeg <= nearTargetBandDeg)
    {
        integrator = 0.0f;
        command = clampf(pTerm + dTerm, -nearTargetOutputLimit, nearTargetOutputLimit);
    }
    else
    {
        command = clampf(command, -outputLimit, outputLimit);
    }

    const float deltaCommand = clampf(command - prevCommand, -maxCommandStep, maxCommandStep);
    command = prevCommand + deltaCommand;

    const float absCommand = fabsf(command);
    if ((absErrorDeg > breakawayErrorDeg) && (absErrorDeg > nearTargetBandDeg))
    {
        if ((command * errorDeg) > 0.0f)
        {
            if ((command > 0.0f) && (absCommand < minEffectiveCommand))
            {
                command = minEffectiveCommand;
            }
            else if ((command < 0.0f) && (absCommand < minEffectiveCommand))
            {
                command = -minEffectiveCommand;
            }
        }
        else if ((command * errorDeg) < 0.0f)
        {
            command = 0.0f;
        }
    }

    prevCommand = command;
    return command;
}

void pid_reset(void)
{
    integrator = 0.0f;
    prevAngle = motor_control_get_angle_deg();
    lastAngleDeg = prevAngle;
    lastTargetAngleDeg = prevAngle;
    lastErrorDeg = 0.0f;
    derivativeFiltered = 0.0f;
    prevCommand = 0.0f;
}

void pid_set_gains(float newKp, float newKi, float newKd)
{
    kp = newKp;
    ki = newKi;
    kd = newKd;
    integrator = 0.0f;
    derivativeFiltered = 0.0f;
}

float pid_get_kp(void)
{
    return kp;
}

float pid_get_ki(void)
{
    return ki;
}

float pid_get_kd(void)
{
    return kd;
}

float pid_get_last_target_angle_deg(void)
{
    return lastTargetAngleDeg;
}

float pid_get_last_angle_deg(void)
{
    return lastAngleDeg;
}

float pid_get_last_error_deg(void)
{
    return lastErrorDeg;
}

float pid_get_last_command(void)
{
    return prevCommand;
}
