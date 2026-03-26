#ifndef HAL_INTERFACE_H
#define HAL_INTERFACE_H

/*
 * hal_interface.h
 * ===========================================================================
 * Hardware abstraction layer — ESP32/Arduino version.
 * Same API as the STM32 version so note_player and midi_parser are unchanged.
 *
 * STM32 → ESP32 translation:
 *   HAL_GetTick()      → millis()
 *   HAL_Delay()        → delay()
 *   HAL_UART_Transmit  → Serial.write()
 *   HAL_UART_Receive   → Serial.readBytes()
 *   volatile ISR flag  → same pattern, works on ESP32
 * ===========================================================================
 */

#include <stdint.h>
#include "Arduino.h"

#ifdef __cplusplus
extern "C" {
#endif

/* --------------------------------------------------------------------------
 * Calibration — EDIT THESE
 * -------------------------------------------------------------------------- */

/* How close is "close enough" to target (mm). */
#define HAL_ARRIVAL_TOLERANCE_MM    0.5f

/* Extra wait after motor arrives before pressing finger (ms).
 * Lets the carriage stop vibrating. Tune on real hardware. */
#define HAL_SETTLE_TIME_MS          50U

/* Fraction of note duration to hold the key. */
#define HAL_PRESS_DURATION_SCALE    0.85f

/* Minimum hold time in ms. */
#define HAL_MIN_PRESS_MS            10U


/* --------------------------------------------------------------------------
 * Initialisation — call once in setup()
 * -------------------------------------------------------------------------- */
void hal_init(void);

/* Called from the 1 kHz timer ISR — runs PID and sets motor speed. */
void hal_pidStep(void);


/* --------------------------------------------------------------------------
 * Motor control
 * -------------------------------------------------------------------------- */
void  hal_motorSetTarget(float positionMM);
float hal_motorGetPosition(void);
int   hal_motorHasArrived(void);
void  hal_motorWaitUntilArrived(void);
void  hal_motorNotifyArrived(void);   /* called internally by hal_pidStep */


/* --------------------------------------------------------------------------
 * Finger control  (index: 0=W1 1=W2 2=W3 3=B1 4=B2)
 * -------------------------------------------------------------------------- */
void hal_fingerPress(uint8_t fingerIndex);
void hal_fingerRelease(uint8_t fingerIndex);
void hal_fingerReleaseAll(void);


/* --------------------------------------------------------------------------
 * Timing
 * -------------------------------------------------------------------------- */
uint32_t hal_getTick(void);      /* millis()  */
void     hal_delay(uint32_t ms); /* delay()   */


#ifdef __cplusplus
}
#endif

#endif /* HAL_INTERFACE_H */
