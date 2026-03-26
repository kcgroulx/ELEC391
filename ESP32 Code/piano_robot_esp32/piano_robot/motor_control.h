#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

/*
 * motor_control.h
 * ===========================================================================
 * ESP32 motor drive (LEDC PWM) and encoder (interrupt-based) interface.
 *
 * PIN ASSIGNMENT — EDIT THESE to match your wiring:
 *
 *   MOTOR_PWM1_PIN / MOTOR_PWM2_PIN
 *     The two half-bridge PWM inputs on your motor driver (e.g. L298N IN1/IN2,
 *     or DRV8833 AIN1/AIN2). Must be PWM-capable GPIO pins.
 *
 *   ENCODER_PIN_A / ENCODER_PIN_B
 *     Quadrature encoder channels. Must support interrupts (all ESP32 GPIOs do
 *     except GPIO 34-39 which are input-only — fine for encoder reading).
 *
 * MECHANICAL CONSTANTS — EDIT THESE:
 *
 *   ENCODER_CPR_MOTOR    pulses per revolution at the motor shaft (check datasheet)
 *   GEAR_RATIO           gearbox ratio (output rev per motor rev, e.g. 1/30 = 30:1)
 *   LINEAR_TRAVEL_PER_REV  mm of carriage travel per output shaft revolution
 *                          = 2 * pi * pulley_radius_mm
 *                          Your STM32 code had 37.7 mm (6 mm pulley radius)
 * ===========================================================================
 */

#include <stdint.h>
#include <math.h>
#include "Arduino.h"

/* --------------------------------------------------------------------------
 * Pin assignments — EDIT THESE
 * -------------------------------------------------------------------------- */
#define MOTOR_PWM1_PIN      25      /* H-bridge input 1 (forward)            */
#define MOTOR_PWM2_PIN      26      /* H-bridge input 2 (reverse)            */
#define ENCODER_PIN_A       34      /* Encoder channel A                     */
#define ENCODER_PIN_B       35      /* Encoder channel B                     */

/* LEDC PWM config */
#define MOTOR_PWM1_CH       0       /* LEDC channel for PWM1 (0–15)          */
#define MOTOR_PWM2_CH       1       /* LEDC channel for PWM2                 */
#define MOTOR_PWM_FREQ      20000   /* 20 kHz — above audible range          */
#define MOTOR_PWM_RES_BITS  8       /* 8-bit resolution → duty 0–255         */
#define MOTOR_PWM_MAX_DUTY  255U

/* --------------------------------------------------------------------------
 * Mechanical constants — EDIT THESE
 * -------------------------------------------------------------------------- */
#define ENCODER_CPR_MOTOR       64.0f   /* encoder counts per motor revolution  */
#define GEAR_RATIO              30.0f   /* motor revs per output shaft rev       */
#define ENCODER_CPR_OUTPUT      (ENCODER_CPR_MOTOR * GEAR_RATIO * 4.0f)
                                        /* ×4 for quadrature decoding            */
#define LINEAR_TRAVEL_PER_REV   37.7f  /* mm per output shaft revolution        */
                                        /* = 2*pi*6mm pulley — confirm this!     */

/* --------------------------------------------------------------------------
 * Public API (matches STM32 version exactly so pid.cpp is unchanged)
 * -------------------------------------------------------------------------- */
void  motor_control_init(void);
void  motor_control_setMotorSpeed(float speed);   /* -1.0 to +1.0 */
void  motor_controller_encoderUpdatePosition(void);
float motor_controller_encoderGetAngleDeg(void);
float motor_controller_encoderGetLinearPosition(void);
void  motor_controller_encoderZeroPosition(void);

#endif /* MOTOR_CONTROL_H */
