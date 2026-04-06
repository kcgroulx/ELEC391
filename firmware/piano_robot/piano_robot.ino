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
#include "midi_parser.h"
#include "song_player.h"
#include "platform_io.h"
#include "config.h"

hw_timer_t* g_pidTimer = NULL;
/* Default ESP32 serial RX buffering is too small for larger MIDI payloads. */
static const size_t SERIAL_RX_BUFFER_BYTES = (size_t)MIDI_UART_BUF_SIZE + 64U;

/* --------------------------------------------------------------------------
 * ISR — sets a flag only. Nothing else. No function calls into flash.
 * IRAM_ATTR is still needed so the ISR *itself* lives in IRAM.
 * -------------------------------------------------------------------------- */
void IRAM_ATTR onPIDTimer(void)
{
    hal_flagPIDPending();   /* sets a volatile flag — safe from ISR */
}


/* --------------------------------------------------------------------------
 * Button helpers — debounced press detection
 * Only active when user_button_pin >= 0 in config.h
 * -------------------------------------------------------------------------- */
static bool g_buttonEnabled = false;
static bool g_robotRunning  = true;

static bool waitForButtonPress(uint32_t timeoutMs)
{
    if (!g_buttonEnabled) return false;
    uint32_t deadline = millis() + timeoutMs;
    while (platform_io_is_user_button_active()) {
        hal_runPendingPID();
        if (timeoutMs > 0 && millis() > deadline) return false;
    }
    while (!platform_io_is_user_button_active()) {
        hal_runPendingPID();
        if (timeoutMs > 0 && millis() > deadline) return false;
    }
    uint32_t pressStart = millis();
    while ((millis() - pressStart) < 20U) {
        hal_runPendingPID();
        if (!platform_io_is_user_button_active()) return false;
    }
    return true;
}

static bool g_btnPrevState = false;
static bool checkButtonEdge(void)
{
    if (!g_buttonEnabled) return false;
    bool cur = platform_io_is_user_button_active();
    bool pressed = (cur && !g_btnPrevState);
    g_btnPrevState = cur;
    return pressed;
}


/* --------------------------------------------------------------------------
 * setup()
 * -------------------------------------------------------------------------- */
void setup(void)
{
    Serial.setRxBufferSize(SERIAL_RX_BUFFER_BYTES);
    Serial.begin(115200);
    Serial.println("Piano robot starting...");

    motor_control_init();
    piano_hal_init();

    /* 1 kHz timer */
    g_pidTimer = timerBegin(1000000);
    timerAttachInterrupt(g_pidTimer, &onPIDTimer);
    timerAlarm(g_pidTimer, 1000, true, 0);

    Serial.println("Hardware initialised.");

    g_buttonEnabled = (app_config::user_button_pin >= 0);

    if (g_buttonEnabled) {
        Serial.println("Press button to home and start (auto-start in 5s)...");
        bool pressed = waitForButtonPress(5000);
        if (pressed) {
            Serial.println("Button pressed — homing...");
        } else {
            Serial.println("Auto-starting — homing...");
        }
    } else {
        Serial.println("No button configured — homing...");
    }

    song_player_homing();
    Serial.println("Homing done. Ready.");
    g_robotRunning = true;
    g_btnPrevState = g_buttonEnabled ? platform_io_is_user_button_active() : false;
}


/* --------------------------------------------------------------------------
 * loop()
 * -------------------------------------------------------------------------- */
void loop(void)
{
    hal_runPendingPID();

    /* ── Safety: check button for stop/start toggle ─────────────────────── */
    if (checkButtonEdge()) {
        if (g_robotRunning) {
            Serial.println("\r\n[SAFETY] Button pressed — STOPPING.");
            motor_control_set_motor_speed(0.0f);
            hal_fingerReleaseAll();
            hal_pidSetEnabled(false);
            g_robotRunning = false;
            Serial.println("Press button again to re-home and restart.");
        } else {
            Serial.println("\r\n[SAFETY] Button pressed — re-homing...");
            hal_pidSetEnabled(true);
            song_player_homing();
            Serial.println("Homing done. Robot running.");
            g_robotRunning = true;
        }
        delay(300);
        g_btnPrevState = platform_io_is_user_button_active();
        return;
    }

    /* ── Safety: far limit switch e-stop ────────────────────────────────── */
    if (hal_isEStopped()) {
        static bool ePrintedOnce = false;
        if (!ePrintedOnce) {
            Serial.println("\r\n[SAFETY] FAR LIMIT SWITCH HIT — motor stopped!");
            if (g_buttonEnabled) {
                Serial.println("Move carriage away from limit, then press button to re-home.");
            } else {
                Serial.println("Move carriage away from limit and reset the ESP32.");
            }
            ePrintedOnce = true;
            g_robotRunning = false;
        }
        if (checkButtonEdge()) {
            hal_clearEStop();
            if (!hal_isEStopped()) {
                Serial.println("Re-homing...");
                song_player_homing();
                Serial.println("Homing done. Robot running.");
                g_robotRunning = true;
                ePrintedOnce = false;
            }
            delay(300);
            g_btnPrevState = platform_io_is_user_button_active();
        }
        return;
    }

    /* ── Normal operation ───────────────────────────────────────────────── */
    if (!g_robotRunning) return;

    /* Uncomment ONE active mode: */
    //test_CmajorScale();           /* 1. C major scale C2–C3                 */
    SongPlayer_run();             /* 2. Receive MIDI over Serial and play   */
    //test_solenoidPairs();         /* 3. Fire all 10 two-finger combos       */
    //test_randomKeys();            /* 4. Random notes, all 5 fingers equally */
    //test_limitSwitchThenPlay();   /* 5. Home via limit switch, then play    */
    //test_fingersSolenoidOnly();   /* 6. Fire each solenoid without moving   */
    //test_allFingers();            /* 7. Move to keys + press all 5 fingers  */
    //test_happyBirthday();         /* 8. Happy Birthday in C major           */
    //test_chords();                /* 9. Chord playback test                 */

}
