/*
 * piano_robot.ino
 * ===========================================================================
 * ESP32 Arduino entry point.
 *
 * FIX: The ISR now only sets a flag — no PID, no motor writes, no flash calls.
 *
 * WHY: ESP32 crashes if an ISR calls any function stored in flash (not IRAM).
 * ledcWrite, pid_step, etc. all live in flash. The only safe thing to do in
 * an ISR is set a volatile flag and return immediately.
 *
 * The PID runs in the main loop via hal_runPendingPID(), which is called:
 *   - In hal_motorWaitUntilArrived() so blocking moves still work
 *   - In loop() so the PID keeps running between songs
 * ===========================================================================
 */

#include "motor_control.h"
#include "hal_interface.h"
#include "song_player.h"

hw_timer_t* g_pidTimer = NULL;

/* --------------------------------------------------------------------------
 * ISR — sets a flag only. Nothing else. No function calls into flash.
 * IRAM_ATTR is still needed so the ISR *itself* lives in IRAM.
 * -------------------------------------------------------------------------- */
void IRAM_ATTR onPIDTimer(void)
{
    hal_flagPIDPending();   /* sets a volatile flag — safe from ISR */
}


/* --------------------------------------------------------------------------
 * setup()
 * -------------------------------------------------------------------------- */
void setup(void)
{
    Serial.begin(115200);
    Serial.println("Piano robot starting...");

    motor_control_init();
    piano_hal_init();

    /* 1 kHz timer */
    g_pidTimer = timerBegin(1000000);
    timerAttachInterrupt(g_pidTimer, &onPIDTimer);
    timerAlarm(g_pidTimer, 1000, true, 0);

    Serial.println("Hardware initialised. Running homing sequence...");
    song_player_homing();
    Serial.println("Homing done. Ready.");

    /* test_printKeyMap(); */
}


/* --------------------------------------------------------------------------
 * loop()
 * Calls hal_runPendingPID() so PID keeps running while waiting for MIDI.
 * -------------------------------------------------------------------------- */
void loop(void)
{
    hal_runPendingPID();   /* keep motor under control between songs */

    /* --- Hardware tests --- */
    //test_CmajorScale();           /* 1. C major scale C2–C3                 */
    test_happyBirthday();         /* 2. Happy Birthday in C major           */
    //test_solenoidPairs();         /* 3. Fire all 10 two-finger combos       */
    //test_randomKeys();            /* 4. Random notes, all 5 fingers equally */
    //test_limitSwitchThenPlay();   /* 3. Home via limit switch, then play    */
    //test_fingersSolenoidOnly();   /* 4. Fire each solenoid without moving   */
    //test_allFingers();            /* 5. Move to keys + press all 5 fingers  */
    //SongPlayer_run();             /* 6. Receive MIDI over Serial and play   */

    while (true) { hal_runPendingPID(); }  /* stop after one pass */
}
