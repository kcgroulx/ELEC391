/*
 * piano_robot.ino
 * ===========================================================================
 * ESP32 Arduino entry point for the robot piano player.
 *
 * SETUP CHECKLIST:
 *   1. Set pin numbers in motor_control.h  (PWM pins, encoder pins)
 *   2. Set pin numbers in song_player.cpp  (homing limit switch pin)
 *   3. Set LINEAR_TRAVEL_PER_REV in motor_control.h (mm per shaft revolution)
 *   4. Set WHITE_KEY_WIDTH_MM in piano_keymap.h (measure your piano)
 *   5. Upload, open Serial Monitor at 115200 baud
 *   6. Call test_printKeyMap() first to verify comms and key positions
 *
 * FLOW:
 *   setup() → initialise hardware, start 1 kHz PID timer
 *   loop()  → run song player (receive MIDI over Serial, play it)
 * ===========================================================================
 */

#include "motor_control.h"
#include "hal_interface.h"
#include "song_player.h"

/* --------------------------------------------------------------------------
 * 1 kHz PID timer
 * The PID runs inside this ISR exactly as it did in TIM4_IRQHandler on STM32.
 * IRAM_ATTR places the function in IRAM so it runs safely from an interrupt.
 * -------------------------------------------------------------------------- */
hw_timer_t* g_pidTimer = NULL;

void IRAM_ATTR onPIDTimer(void)
{
    motor_controller_encoderUpdatePosition();
    hal_pidStep();   /* runs cascaded_control_step and sets motor speed */
}


/* --------------------------------------------------------------------------
 * setup()
 * -------------------------------------------------------------------------- */
void setup(void)
{
    Serial.begin(115200);
    Serial.println("Piano robot starting...");

    /* Initialise motor PWM and encoder */
    motor_control_init();

    /* Initialise HAL (clears arrived flag, etc.) */
    hal_init();

    /* Start 1 kHz PID timer
     *   timerBegin(unit, prescaler, countUp)
     *   ESP32 base clock = 80 MHz
     *   prescaler = 80 → timer ticks at 1 MHz
     *   alarm at 1000 ticks → fires every 1 ms = 1 kHz
     */
    g_pidTimer = timerBegin(0, 80, true);
    timerAttachInterrupt(g_pidTimer, &onPIDTimer, true);
    timerAlarmWrite(g_pidTimer, 1000, true);
    timerAlarmEnable(g_pidTimer);

    Serial.println("Hardware initialised. Running homing sequence...");
    song_player_homing();
    Serial.println("Homing done. Ready.");

    /* Optional: print key map to verify positions */
    /* test_printKeyMap(); */
}


/* --------------------------------------------------------------------------
 * loop()
 * -------------------------------------------------------------------------- */
void loop(void)
{
    /* Waits for a MIDI file over Serial, parses it, plays it.
     * To test without MIDI: call test_playScale() instead. */
    SongPlayer_run();
}
