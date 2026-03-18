/* hal_interface.c
 * -----------------------------------------------------------------------
 * Concrete implementation of hal_interface.h for STM32F103 + your
 * motor_control / pid setup.
 *
 * HOW THIS FITS INTO YOUR EXISTING CODE:
 *
 *   stm32f1xx_it.c  (you edit this — see note at bottom of file)
 *     TIM4_IRQHandler already runs at 1 kHz.
 *     You need to:
 *       1. Remove the test target_sequence_deg block.
 *       2. Stop writing target_angle inside the IRQ — hal_motorSetTarget()
 *          does that from the main loop now.
 *       3. Call hal_motorNotifyArrived() when settled_ticks fires.
 *
 *   main.c
 *     No changes needed.
 *
 *   song_player.c  (Step 2 note playback)
 *     Calls hal_motorSetTarget(), hal_motorWaitUntilArrived(),
 *     hal_fingerPress(), hal_fingerRelease().
 *
 * UNIT CONVERSION:
 *   Your PID works in degrees.
 *   motor_controller_encoderGetLinearPosition() already gives mm.
 *   To convert mm → degrees for the PID target:
 *     degrees = (mm / LINEAR_TRAVEL_PER_REV) * 360.0
 *   LINEAR_TRAVEL_PER_REV is defined in motor_control.h — make sure it
 *   matches your actual hardware.
 * -----------------------------------------------------------------------
 */

#include "hal_interface.h"
#include "main.h"
#include "motor_control.h"

/* target_angle and motor_command are declared in stm32f1xx_it.c.
 * We access them here via extern. */
extern float target_angle;
extern float motor_command;

/* -----------------------------------------------------------------------
 * Internal state
 * ----------------------------------------------------------------------- */

/* Set to 1 by hal_motorNotifyArrived() (called from IRQ).
 * Cleared to 0 by hal_motorSetTarget() (called from main loop).
 * Declared volatile because it is written in an ISR and read in main. */
static volatile int s_motorArrived = 0;

/* Last target we were commanded to (mm). Used for arrival check. */
static volatile float s_targetMM = 0.0f;


/* -----------------------------------------------------------------------
 * Conversion helper
 *
 * mm_to_deg: converts a carriage position in mm to the motor shaft angle
 * in degrees that the PID expects.
 *
 * Formula:  degrees = (mm / LINEAR_TRAVEL_PER_REV) * 360.0
 *
 * LINEAR_TRAVEL_PER_REV is defined in motor_control.h — it is the number
 * of mm the carriage moves per one full shaft revolution.
 * ----------------------------------------------------------------------- */
static float mm_to_deg(float mm)
{
    return (mm / LINEAR_TRAVEL_PER_REV) * 360.0f;
}


/* -----------------------------------------------------------------------
 * Motor control
 * ----------------------------------------------------------------------- */

void hal_motorSetTarget(float positionMM)
{
    s_targetMM    = positionMM;
    s_motorArrived = 0;                 /* clear flag — not arrived yet */
    target_angle  = mm_to_deg(positionMM);  /* write PID target in degrees */
}

float hal_motorGetPosition(void)
{
    /* motor_controller_encoderGetLinearPosition() already returns mm */
    return motor_controller_encoderGetLinearPosition();
}

int hal_motorHasArrived(void)
{
    return s_motorArrived;
}

void hal_motorWaitUntilArrived(void)
{
    /* Spin in the main loop until the IRQ sets the arrived flag.
     * The PID keeps running in TIM4_IRQHandler the whole time. */
    while (!s_motorArrived)
    {
        /* nothing — PID runs in background via TIM4 IRQ */
    }
}

/* Called from TIM4_IRQHandler when settled_ticks fires.
 * Runs in ISR context — keep it short. */
void hal_motorNotifyArrived(void)
{
    s_motorArrived = 1;
}


/* -----------------------------------------------------------------------
 * Finger control
 *
 * pressFinger / releaseFinger are not implemented yet.
 * Add your servo / solenoid control here when the hardware is ready.
 * The index mapping is:  0=W1  1=W2  2=W3  3=B1  4=B2
 * ----------------------------------------------------------------------- */

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
    for (i = 0; i < 5U; i++)
    {
        hal_fingerRelease(i);
    }
}


/* -----------------------------------------------------------------------
 * Timing
 * ----------------------------------------------------------------------- */

uint32_t hal_getTick(void)
{
    return HAL_GetTick();
}

void hal_delay(uint32_t ms)
{
    HAL_Delay(ms);
}


/* =======================================================================
 * REQUIRED EDIT TO stm32f1xx_it.c
 * -----------------------------------------------------------------------
 * Your TIM4_IRQHandler currently contains a test sequence that writes
 * target_angle on every tick. You need to remove that block and add one
 * call to hal_motorNotifyArrived().
 *
 * Replace your current TIM4_IRQHandler USER CODE section with this:
 *
 *
 *   void TIM4_IRQHandler(void)
 *   {
 *       HAL_TIM_IRQHandler(&htim4);
 *
 *       // USER CODE BEGIN TIM4_IRQn 1
 *
 *       motor_controller_encoderUpdatePosition();
 *       actual_angle = motor_controller_encoderGetAngleDeg();
 *
 *       // --- REMOVED: test target_sequence_deg block ---
 *       // target_angle is now set by hal_motorSetTarget() from main loop.
 *
 *       static uint16_t settled_ticks = 0;
 *       const float     reach_tolerance_deg  = 3.0f;
 *       const uint16_t  settled_ticks_required = 100;
 *
 *       float angle_error     = target_angle - actual_angle;
 *       float abs_angle_error = (angle_error < 0.0f) ? -angle_error : angle_error;
 *
 *       if (abs_angle_error <= reach_tolerance_deg)
 *       {
 *           if (++settled_ticks >= settled_ticks_required)
 *           {
 *               settled_ticks = 0;
 *               hal_motorNotifyArrived();   // <-- ADD THIS LINE
 *           }
 *       }
 *       else
 *       {
 *           settled_ticks = 0;
 *       }
 *
 *       motor_command = cascaded_control_step(target_angle);
 *       motor_control_setMotorSpeed(motor_command);  // <-- uncomment this too
 *
 *       // USER CODE END TIM4_IRQn 1
 *   }
 *
 * ======================================================================= */
