/*
 * hal_interface.cpp
 * ===========================================================================
 * ESP32/Arduino implementation of the hardware abstraction layer.
 * ===========================================================================
 */

#include "hal_interface.h"
#include "motor_control.h"
#include "pid.h"

/* --------------------------------------------------------------------------
 * Internal state
 * -------------------------------------------------------------------------- */

/* PID target in degrees — set by hal_motorSetTarget, read by hal_pidStep */
static volatile float s_targetDeg    = 0.0f;

/* Arrived flag — set by hal_motorNotifyArrived, cleared by hal_motorSetTarget */
static volatile int   s_motorArrived = 0;

/* Settled tick counter — same logic as STM32 version */
static volatile uint16_t s_settledTicks = 0;

static const float  REACH_TOLERANCE_DEG    = 3.0f;
static const uint16_t SETTLED_TICKS_REQUIRED = 100U;  /* 100 ms at 1 kHz */

/* --------------------------------------------------------------------------
 * Conversion: mm → degrees (same formula as STM32 version)
 * -------------------------------------------------------------------------- */
static float mm_to_deg(float mm)
{
    return (mm / LINEAR_TRAVEL_PER_REV) * 360.0f;
}

/* --------------------------------------------------------------------------
 * hal_init
 * -------------------------------------------------------------------------- */
void hal_init(void)
{
    s_targetDeg    = 0.0f;
    s_motorArrived = 1;   /* start as arrived so first move triggers cleanly */
    s_settledTicks = 0;
}

/* --------------------------------------------------------------------------
 * hal_pidStep — called from 1 kHz timer ISR in piano_robot.ino
 *
 * Runs the PID and checks for arrival — equivalent to TIM4_IRQHandler
 * on the STM32.
 * -------------------------------------------------------------------------- */
void IRAM_ATTR hal_pidStep(void)
{
    float angle      = motor_controller_encoderGetAngleDeg();
    float error      = s_targetDeg - angle;
    float abs_error  = (error < 0.0f) ? -error : error;

    /* Settled tick logic — same as STM32 */
    if (abs_error <= REACH_TOLERANCE_DEG) {
        if (++s_settledTicks >= SETTLED_TICKS_REQUIRED) {
            s_settledTicks = 0;
            hal_motorNotifyArrived();
        }
    } else {
        s_settledTicks = 0;
    }

    /* Run PID and drive motor */
    float command = cascaded_control_step(s_targetDeg);
    motor_control_setMotorSpeed(command);
}

/* --------------------------------------------------------------------------
 * Motor control
 * -------------------------------------------------------------------------- */

void hal_motorSetTarget(float positionMM)
{
    s_motorArrived = 0;
    s_settledTicks = 0;
    s_targetDeg    = mm_to_deg(positionMM);
}

float hal_motorGetPosition(void)
{
    return motor_controller_encoderGetLinearPosition();
}

int hal_motorHasArrived(void)
{
    return s_motorArrived;
}

void hal_motorWaitUntilArrived(void)
{
    /* Spin — PID keeps running in the background via timer ISR */
    while (!s_motorArrived) { /* nothing */ }
}

void IRAM_ATTR hal_motorNotifyArrived(void)
{
    s_motorArrived = 1;
}

/* --------------------------------------------------------------------------
 * Finger control
 * Stubs — fill in servo/solenoid control when hardware is ready.
 * -------------------------------------------------------------------------- */

void hal_fingerPress(uint8_t fingerIndex)
{
    /* TODO: drive servo or solenoid for fingerIndex */
    (void)fingerIndex;
}

void hal_fingerRelease(uint8_t fingerIndex)
{
    /* TODO: release servo or solenoid for fingerIndex */
    (void)fingerIndex;
}

void hal_fingerReleaseAll(void)
{
    uint8_t i;
    for (i = 0; i < 5U; i++) hal_fingerRelease(i);
}

/* --------------------------------------------------------------------------
 * Timing
 * -------------------------------------------------------------------------- */

uint32_t hal_getTick(void)  { return (uint32_t)millis(); }
void     hal_delay(uint32_t ms) { delay(ms); }
