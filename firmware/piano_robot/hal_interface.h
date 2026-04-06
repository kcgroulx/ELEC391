#ifndef HAL_INTERFACE_H
#define HAL_INTERFACE_H

/*
 * hal_interface.h
 * ===========================================================================
 * Hardware abstraction layer — ESP32/Arduino version.
 *
 * PID TIMING MODEL:
 *   A 1 kHz hw_timer ISR calls hal_flagPIDPending() — this is the ONLY thing
 *   safe to do from an ISR on ESP32 (sets a volatile flag, no flash access).
 *
 *   hal_runPendingPID() checks that flag and does the real work: encoder read,
 *   PID compute, motor write. Call it from:
 *     - hal_motorWaitUntilArrived()  (already done — blocking moves work)
 *     - loop()                       (keeps motor live between songs)
 * ===========================================================================
 */

#include <stdint.h>
#include "Arduino.h"

#ifdef __cplusplus
extern "C" {
#endif

/* --------------------------------------------------------------------------
 * Calibration
 * -------------------------------------------------------------------------- */
// MUST be >= PID deadband in mm (8deg * 40mm/360 = 0.89mm). Use 1.5mm for margin.
#define HAL_ARRIVAL_TOLERANCE_MM    1.5f
#define HAL_SETTLE_TIME_MS          0U
#define HAL_PRESS_DURATION_SCALE    0.85f
#define HAL_MIN_PRESS_MS            10U


/* --------------------------------------------------------------------------
 * Initialisation
 * -------------------------------------------------------------------------- */
void piano_hal_init(void);


/* --------------------------------------------------------------------------
 * PID timing — two-step pattern to avoid ISR/flash crash
 * -------------------------------------------------------------------------- */

/* Disable/enable PID — use during homing so open-loop drive works */
void hal_pidSetEnabled(bool enabled);

/* Emergency stop — latched by far-side limit switch, cleared manually */
int  hal_isEStopped(void);
void hal_clearEStop(void);

/* Called from ISR — ONLY sets a volatile flag. Safe. */
void hal_flagPIDPending(void);

/* Called from main loop — does the actual PID work (encoder, compute, drive).
 * Returns immediately if no tick is pending. */
void hal_runPendingPID(void);


/* --------------------------------------------------------------------------
 * Motor control
 * -------------------------------------------------------------------------- */
void  hal_motorSetTarget(float positionMM);
float hal_motorGetPosition(void);
int   hal_motorHasArrived(void);
void  hal_motorWaitUntilArrived(void);
void  hal_motorNotifyArrived(void);


/* --------------------------------------------------------------------------
 * Finger control  (0=W1 1=W2 2=W3 3=B1 4=B2)
 * -------------------------------------------------------------------------- */
void hal_fingerPress(uint8_t fingerIndex);
void hal_fingerRelease(uint8_t fingerIndex);
void hal_fingerReleaseAll(void);


/* --------------------------------------------------------------------------
 * Timing
 * -------------------------------------------------------------------------- */
uint32_t hal_getTick(void);
void     hal_delay(uint32_t ms);


#ifdef __cplusplus
}
#endif

#endif /* HAL_INTERFACE_H */
