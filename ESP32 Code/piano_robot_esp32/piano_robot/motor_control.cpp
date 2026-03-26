/*
 * motor_control.cpp
 * ===========================================================================
 * ESP32 implementation of motor drive and encoder positioning.
 *
 * STM32 → ESP32 translation:
 *   TIM2/TIM3 PWM (HAL)        → LEDC peripheral (ledcWrite)
 *   TIM1 hardware encoder       → GPIO interrupts (quadrature decode in ISR)
 *   __HAL_TIM_GET_COUNTER()     → volatile counter incremented in ISR
 *   __HAL_TIM_SET_COMPARE()     → ledcWrite()
 *
 * QUADRATURE DECODING:
 *   Both encoder pins trigger an interrupt on CHANGE.
 *   On each edge, we read both pins to determine direction:
 *     A==B → forward (+1)
 *     A!=B → reverse (-1)
 *   This gives ×2 decoding. ENCODER_CPR_OUTPUT accounts for this.
 * ===========================================================================
 */

#include "motor_control.h"

/* --------------------------------------------------------------------------
 * Encoder state — written in ISR, read in main/PID context.
 * volatile prevents compiler from caching these in a register.
 * -------------------------------------------------------------------------- */
static volatile int32_t s_enc_position_counts = 0;
static volatile uint8_t s_enc_last_a          = 0;

/* --------------------------------------------------------------------------
 * Encoder ISR
 * Called on every edge of either encoder channel.
 * IRAM_ATTR places this in IRAM so it runs reliably from an interrupt.
 * -------------------------------------------------------------------------- */
static void IRAM_ATTR encoderISR(void)
{
    uint8_t a = (uint8_t)digitalRead(ENCODER_PIN_A);
    uint8_t b = (uint8_t)digitalRead(ENCODER_PIN_B);

    /* Direction: if A and B match → same direction as last A edge → forward
     *            if A and B differ → direction changed → reverse            */
    if (a == b) {
        s_enc_position_counts++;
    } else {
        s_enc_position_counts--;
    }
    s_enc_last_a = a;
}

/* --------------------------------------------------------------------------
 * motor_control_init
 * -------------------------------------------------------------------------- */
void motor_control_init(void)
{
    /* Set up LEDC PWM for both half-bridge channels */
    ledcSetup(MOTOR_PWM1_CH, MOTOR_PWM_FREQ, MOTOR_PWM_RES_BITS);
    ledcSetup(MOTOR_PWM2_CH, MOTOR_PWM_FREQ, MOTOR_PWM_RES_BITS);
    ledcAttachPin(MOTOR_PWM1_PIN, MOTOR_PWM1_CH);
    ledcAttachPin(MOTOR_PWM2_PIN, MOTOR_PWM2_CH);

    /* Start with motor stopped */
    motor_control_setMotorSpeed(0.0f);

    /* Set up encoder pins with internal pull-ups */
    pinMode(ENCODER_PIN_A, INPUT_PULLUP);
    pinMode(ENCODER_PIN_B, INPUT_PULLUP);

    /* Attach interrupt to both channels for ×2 quadrature decoding */
    attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A), encoderISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_B), encoderISR, CHANGE);

    s_enc_position_counts = 0;
    s_enc_last_a = (uint8_t)digitalRead(ENCODER_PIN_A);
}

/* --------------------------------------------------------------------------
 * motor_control_setMotorSpeed
 * speed: -1.0 (full reverse) to +1.0 (full forward), 0.0 = stop
 * -------------------------------------------------------------------------- */
void motor_control_setMotorSpeed(float speed)
{
    /* Clamp */
    if (speed >  1.0f) speed =  1.0f;
    if (speed < -1.0f) speed = -1.0f;

    uint8_t duty = (uint8_t)(fabsf(speed) * (float)MOTOR_PWM_MAX_DUTY);

    if (speed > 0.0f) {
        ledcWrite(MOTOR_PWM1_CH, duty);
        ledcWrite(MOTOR_PWM2_CH, 0);
    } else if (speed < 0.0f) {
        ledcWrite(MOTOR_PWM1_CH, 0);
        ledcWrite(MOTOR_PWM2_CH, duty);
    } else {
        ledcWrite(MOTOR_PWM1_CH, 0);
        ledcWrite(MOTOR_PWM2_CH, 0);
    }
}

/* --------------------------------------------------------------------------
 * Encoder position functions
 * On ESP32 the encoder ISR accumulates counts directly — no need to call
 * motor_controller_encoderUpdatePosition() to read a hardware timer.
 * We keep the function for API compatibility (piano_robot.ino calls it from
 * the PID timer ISR — it's a no-op here but harmless).
 * -------------------------------------------------------------------------- */
void motor_controller_encoderUpdatePosition(void)
{
    /* No-op on ESP32: counts are updated in encoderISR() automatically. */
}

float motor_controller_encoderGetAngleDeg(void)
{
    /* Atomically read the volatile counter.
     * noInterrupts/interrupts ensure we don't read a half-updated 32-bit value. */
    noInterrupts();
    int32_t counts = s_enc_position_counts;
    interrupts();

    return ((float)counts / ENCODER_CPR_OUTPUT) * 360.0f;
}

float motor_controller_encoderGetLinearPosition(void)
{
    return (motor_controller_encoderGetAngleDeg() / 360.0f) * LINEAR_TRAVEL_PER_REV;
}

void motor_controller_encoderZeroPosition(void)
{
    noInterrupts();
    s_enc_position_counts = 0;
    interrupts();
}
