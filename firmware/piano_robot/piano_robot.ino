/*
 * piano_robot.ino
 * ===========================================================================
 * ESP32 Arduino entry point.
 *
 * The 1 kHz timer ISR only sets a flag. All PID and playback work runs in the
 * main loop so playback, pause, and fault recovery can share the same logic.
 * ===========================================================================
 */

#include "motor_control.h"
#include "hal_interface.h"
#include "midi_parser.h"
#include "song_player.h"
#include "platform_io.h"
#include "config.h"

hw_timer_t* g_pidTimer = NULL;
static const size_t SERIAL_RX_BUFFER_BYTES = (size_t)MIDI_UART_BUF_SIZE + 64U;

void IRAM_ATTR onPIDTimer(void)
{
    hal_flagPIDPending();
}

void setup(void)
{
    Serial.setRxBufferSize(SERIAL_RX_BUFFER_BYTES);
    Serial.begin(115200);
    Serial.println("Piano robot starting...");

    motor_control_init();
    piano_hal_init();

    g_pidTimer = timerBegin(1000000);
    timerAttachInterrupt(g_pidTimer, &onPIDTimer);
    timerAlarm(g_pidTimer, 1000, true, 0);

    Serial.println("Hardware initialised.");
    Serial.println("Homing...");
    song_player_homing();
    Serial.println("Homing done. Ready.");
}

void loop(void)
{
    hal_runPendingPID();

    /* Uncomment ONE active mode: */
    //test_CmajorScale();
    SongPlayer_run();
    //test_solenoidPairs();
    //test_randomKeys();
    //test_limitSwitchThenPlay();
    //test_fingersSolenoidOnly();
    //test_allFingers();
    //test_happyBirthday();
    //test_chords();
    //test_clocks();
}
