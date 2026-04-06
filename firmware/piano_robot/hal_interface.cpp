/*
 * hal_interface.cpp
 * ===========================================================================
 * ESP32/Arduino hardware abstraction layer.
 *
 * ROOT CAUSE OF CRASH:
 *   ESP32 caches flash in 32-byte pages. When an ISR fires mid-cache-miss,
 *   any call into flash from the ISR causes an EXCVADDR fault and reboot.
 *   ledcWrite, pid_step, Serial.print — all live in flash, all unsafe in ISR.
 *
 * FIX — two-step PID pattern:
 *   1. ISR calls hal_flagPIDPending() — sets one volatile bool. That's it.
 *   2. hal_runPendingPID() — called from main loop — checks the flag and does
 *      all the real work: encoder read, PID, ledcWrite, arrival check.
 *
 * hal_motorWaitUntilArrived() calls hal_runPendingPID() in its spin loop so
 * blocking note playback still gets 1 kHz PID updates.
 * ===========================================================================
 */

#include "hal_interface.h"
#include "motor_control.h"
#include "platform_io.h"
#include "config.h"
#include "pid.h"
#include "Arduino.h"
#include <math.h>
#include <stdio.h>

/* --------------------------------------------------------------------------
 * Internal state
 * -------------------------------------------------------------------------- */

/* Set by ISR (IRAM), cleared by hal_runPendingPID() (flash). Safe because
 * the ISR only writes and the main loop only reads-then-clears. */
static volatile bool     s_pidPending   = false;
static volatile bool     s_pidDisabled  = false;
static volatile bool     s_eStop        = false;   /* emergency stop active */

static volatile float    s_targetMM     = 0.0f;
static volatile int      s_motorArrived = 0;
static volatile uint16_t s_settledTicks = 0;

// Reduced from 100 to 20 ticks: motor is already stopped inside deadband,
// we just need brief confirmation it is not oscillating through the window.
static const uint16_t SETTLED_TICKS_REQUIRED = 0U;     /* no extra stable ticks */

static uint8_t s_fingerMask = 0;

/* --------------------------------------------------------------------------
 * Periodic telemetry — fires every TELEMETRY_PERIOD_TICKS PID ticks.
 * At 1 kHz that's every 1 second.
 * -------------------------------------------------------------------------- */
static const uint32_t TELEMETRY_PERIOD_TICKS = 1000U;
static uint32_t       s_telemetryTick = 0U;

static bool hal_isStrikeReady(float currentMM)
{
    const float errorMM = fabsf(s_targetMM - currentMM);
    const float angleErrorDeg = fabsf(pid_get_last_error_deg());
    const float velocityDegPerSec = fabsf(pid_get_last_velocity_deg_per_sec());

    return (errorMM <= HAL_ARRIVAL_TOLERANCE_MM)
        && (angleErrorDeg <= pid_get_deadband_deg())
        && (velocityDegPerSec <= pid_get_settle_velocity_deg_per_sec());
}

static void hal_printTelemetry(void)
{
    char buf[96];
    float posMM    = motor_control_get_linear_position();
    float targetMM = s_targetMM;
    float errorMM  = targetMM - posMM;

    /* State label */
    const char* state;
    if (s_motorArrived) {
        state = "IDLE    ";
    } else if (s_settledTicks > 0) {
        state = "SETTLING";
    } else {
        state = "MOVING  ";
    }

    /* Compact single-line summary — always printed */
    snprintf(buf, sizeof(buf),
        "[TEL] %s  pos=%6.2fmm  tgt=%6.2fmm  err=%+6.2fmm  enc=%ld  cmd=%+5.3f  fingers=0x%02X\r\n",
        state,
        (double)posMM,
        (double)targetMM,
        (double)errorMM,
        (long)platform_io_get_encoder_count(),
        (double)pid_get_last_command(),
        (unsigned)s_fingerMask);
    Serial.print(buf);

    /* Extended PID detail — only while motor is moving so it doesn't spam when idle */
    if (!s_motorArrived) {
        snprintf(buf, sizeof(buf),
            "       pid_err=%+7.2fdeg  pid_tgt=%8.2fdeg  pid_act=%8.2fdeg  settling=%u/%u ticks\r\n",
            (double)pid_get_last_error_deg(),
            (double)pid_get_last_target_angle_deg(),
            (double)pid_get_last_angle_deg(),
            (unsigned)s_settledTicks,
            (unsigned)SETTLED_TICKS_REQUIRED);
        Serial.print(buf);

        snprintf(buf, sizeof(buf),
            "       kp=%.4f  ki=%.4f  kd=%.4f\r\n",
            (double)pid_get_kp(),
            (double)pid_get_ki(),
            (double)pid_get_kd());
        Serial.print(buf);
    }
}


/* --------------------------------------------------------------------------
 * piano_hal_init
 * -------------------------------------------------------------------------- */
void hal_pidSetEnabled(bool enabled)
{
    s_pidDisabled = !enabled;
    if (enabled) {
        s_targetMM     = 0.0f;
        s_motorArrived = 1;
        s_settledTicks = 0;
        pid_reset();
    }
}

int hal_isEStopped(void)
{
    return s_eStop ? 1 : 0;
}

void hal_clearEStop(void)
{
    /* Only clear if the far limit switch is no longer active */
    if (!platform_io_is_far_limit_active()) {
        s_eStop = false;
        s_motorArrived = 1;
        s_settledTicks = 0;
        pid_reset();
        Serial.println("[SAFETY] E-stop cleared.");
    } else {
        Serial.println("[SAFETY] Cannot clear — far limit switch still active!");
    }
}

void piano_hal_init(void)
{
    s_pidPending   = false;
    s_pidDisabled  = false;
    s_eStop        = false;
    s_targetMM     = 0.0f;
    s_motorArrived = 1;
    s_settledTicks = 0;
    s_fingerMask   = 0;
    s_telemetryTick = 0;
    pid_reset();
}


/* --------------------------------------------------------------------------
 * hal_flagPIDPending  — called from ISR
 *
 * IRAM_ATTR: this function lives in IRAM so the ISR can call it safely.
 * It does nothing except set a bool — no flash access whatsoever.
 * -------------------------------------------------------------------------- */
void IRAM_ATTR hal_flagPIDPending(void)
{
    s_pidPending = true;
}


/* --------------------------------------------------------------------------
 * hal_runPendingPID  — called from main loop
 *
 * Does the real PID work. Lives in flash — that's fine, we're in main context.
 * Returns immediately if no tick is pending so it's cheap to call often.
 * -------------------------------------------------------------------------- */
void hal_runPendingPID(void)
{
    if (!s_pidPending) return;
    s_pidPending = false;

    /* 0. Safety: far-side limit switch kills motor immediately */
    if (platform_io_is_far_limit_active()) {
        motor_control_set_motor_speed(0.0f);
        hal_fingerReleaseAll();
        s_eStop = true;
        s_motorArrived = 1;
        return;
    }

    /* If e-stop is latched, keep motor dead until cleared */
    if (s_eStop) {
        motor_control_set_motor_speed(0.0f);
        return;
    }

    /* 1. Update encoder accumulator (always, even during homing) */
    motor_control_update_encoder();

    /* If PID is disabled (homing), skip everything else so open-loop drive works */
    if (s_pidDisabled) return;

    /* 2. Compute the next control output */
    float currentMM = motor_control_get_linear_position();
    float command = pid_step(s_targetMM);
    motor_control_set_motor_speed(command);

    /* 3. Strike-ready check: position + low velocity for several PID ticks */
    if (hal_isStrikeReady(currentMM)) {
        if (SETTLED_TICKS_REQUIRED == 0U ||
            ++s_settledTicks >= SETTLED_TICKS_REQUIRED) {
            s_settledTicks = 0;
            hal_motorNotifyArrived();
        }
    } else {
        s_settledTicks = 0;
    }

    /* 4. Periodic telemetry — every TELEMETRY_PERIOD_TICKS ticks (1 s at 1 kHz) */
    if (++s_telemetryTick >= TELEMETRY_PERIOD_TICKS) {
        s_telemetryTick = 0;
        hal_printTelemetry();
    }
}


/* --------------------------------------------------------------------------
 * Motor control
 * -------------------------------------------------------------------------- */

void hal_motorSetTarget(float positionMM)
{
    s_motorArrived = 0;
    s_settledTicks = 0;
    s_targetMM     = positionMM;
    pid_reset();
}

float hal_motorGetPosition(void)
{
    return motor_control_get_linear_position();
}

int hal_motorHasArrived(void)
{
    return s_motorArrived;
}

void hal_motorWaitUntilArrived(void)
{
    /* Spin, but run pending PID ticks so the motor keeps moving.
     * Without this the motor would freeze the moment we enter the spin. */
    while (!s_motorArrived) {
        hal_runPendingPID();
    }
}

void hal_motorNotifyArrived(void)
{
    s_motorArrived = 1;
}


/* --------------------------------------------------------------------------
 * Finger control
 * -------------------------------------------------------------------------- */

void hal_fingerPress(uint8_t fingerIndex)
{
    if (fingerIndex >= (uint8_t)app_config::finger_count) return;
    s_fingerMask |= (uint8_t)(1U << fingerIndex);
    platform_io_set_fingers(s_fingerMask);
}

void hal_fingerRelease(uint8_t fingerIndex)
{
    if (fingerIndex >= (uint8_t)app_config::finger_count) return;
    s_fingerMask &= (uint8_t)(~(1U << fingerIndex));
    platform_io_set_fingers(s_fingerMask);
}

void hal_fingerReleaseAll(void)
{
    s_fingerMask = 0;
    platform_io_set_fingers(0x00);
}


/* --------------------------------------------------------------------------
 * Timing
 * -------------------------------------------------------------------------- */

uint32_t hal_getTick(void)      { return (uint32_t)millis(); }
void     hal_delay(uint32_t ms)
{
    /* Run PID during delays so the motor stays under control while waiting */
    uint32_t start = millis();
    while ((millis() - start) < ms) {
        hal_runPendingPID();
    }
}
