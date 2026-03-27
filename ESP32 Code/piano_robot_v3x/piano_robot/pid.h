#ifndef PID_H
#define PID_H

/*
 * pid.h
 * ===========================================================================
 * PID controller for motor position control.
 * Logic is identical to the STM32 version — no platform changes needed.
 * ===========================================================================
 */

#ifdef __cplusplus
extern "C" {
#endif

/* Single-loop position PID. Returns motor command (-1.0 to +1.0).
 * Call from the 1 kHz timer ISR. */
float cascaded_control_step(float target_angle_deg);

#ifdef __cplusplus
}
#endif

#endif /* PID_H */
