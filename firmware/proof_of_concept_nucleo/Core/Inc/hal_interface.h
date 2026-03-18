#ifndef HAL_INTERFACE_H
#define HAL_INTERFACE_H

/* hal_interface.h
 * -----------------------------------------------------------------------
 * Thin abstraction between piano logic and STM32 hardware.
 * All piano code calls these functions — nothing calls HAL directly.
 *
 * CALIBRATION — edit these values to match your hardware:
 *   HAL_LINEAR_TRAVEL_PER_REV_MM  how many mm the carriage moves per full
 *                                 motor shaft revolution. Measure by commanding
 *                                 a known angle and measuring carriage travel.
 *   HAL_ARRIVAL_TOLERANCE_MM      how close is "close enough" to target (mm).
 *   HAL_SETTLE_TIME_MS            extra wait after arrival before pressing.
 *   HAL_PRESS_DURATION_SCALE      fraction of note duration to hold key down.
 * -----------------------------------------------------------------------
 */

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* -----------------------------------------------------------------------
 * Calibration — EDIT THESE to match your hardware
 * ----------------------------------------------------------------------- */

/* How many mm the carriage travels per one full motor revolution.
 * Must match LINEAR_TRAVEL_PER_REV defined in motor_control.h.
 * Set to 1.0 until measured — positions effectively become degrees until then. */
#define HAL_LINEAR_TRAVEL_PER_REV_MM    1.0f    /* TODO: measure your hardware */

/* Motor is "arrived" when within this distance of the target (mm). */
#define HAL_ARRIVAL_TOLERANCE_MM        0.5f

/* Extra wait after arrival before pressing a finger (ms).
 * Lets the carriage stop vibrating. Tune on real hardware. */
#define HAL_SETTLE_TIME_MS              50U

/* Fraction of note duration to hold the key down.
 * Slightly under 1.0 so the key releases cleanly before the next note. */
#define HAL_PRESS_DURATION_SCALE        0.85f

/* Minimum hold time in ms regardless of note length. */
#define HAL_MIN_PRESS_MS                10U


/* -----------------------------------------------------------------------
 * Motor control
 * ----------------------------------------------------------------------- */

/* Command the carriage to move to positionMM (mm from encoder zero).
 * Converts mm → degrees internally and writes to target_angle in stm32f1xx_it.c.
 * Clears the arrived flag so hal_motorHasArrived() returns 0 until settled. */
void hal_motorSetTarget(float positionMM);

/* Returns the current carriage position in mm.
 * Wraps motor_controller_encoderGetLinearPosition(). */
float hal_motorGetPosition(void);

/* Returns 1 when the carriage has settled within HAL_ARRIVAL_TOLERANCE_MM.
 * Flag is set by hal_motorNotifyArrived(), called from TIM4_IRQHandler. */
int hal_motorHasArrived(void);

/* Blocking spin until hal_motorHasArrived() == 1.
 * Safe to call from the main loop — do NOT call from inside an ISR. */
void hal_motorWaitUntilArrived(void);

/* Call this from TIM4_IRQHandler once settled_ticks >= settled_ticks_required.
 * Sets the internal flag that hal_motorHasArrived() reads. */
void hal_motorNotifyArrived(void);


/* -----------------------------------------------------------------------
 * Finger control
 * fingerIndex:  0 = W1  1 = W2  2 = W3  3 = B1  4 = B2
 * ----------------------------------------------------------------------- */

/* Press a finger down (non-blocking at HAL level). */
void hal_fingerPress(uint8_t fingerIndex);

/* Release a finger (non-blocking at HAL level). */
void hal_fingerRelease(uint8_t fingerIndex);

/* Release all five fingers immediately. */
void hal_fingerReleaseAll(void);


/* -----------------------------------------------------------------------
 * Timing
 * ----------------------------------------------------------------------- */

/* Milliseconds since boot. Wraps at ~49 days — fine for our purposes. */
uint32_t hal_getTick(void);

/* Blocking delay in milliseconds. */
void hal_delay(uint32_t ms);


#ifdef __cplusplus
}
#endif

#endif /* HAL_INTERFACE_H */
