/**
 * @file song_player.c
 * @brief Logic for song playing
 *
 * Intended for use with STM32 platform
 */

/* Includes */
#include "song_player.h"
#include "main.h"
#include "motor_control.h"
#include "note_player.h"
#include "piano_keymap.h"
#include <stdio.h>
#include <string.h>
#include "usart.h"
#include "midi_parser.h"

static uint8_t   uartBuf[MIDI_UART_BUF_SIZE];
static NoteEvent events[MIDI_MAX_NOTES];

/* Public Function Definitions */
void SongPlayer_run(void) {
    MidiParseResult result;
    Midi_receiveAndParse(uartBuf, sizeof(uartBuf), events, MIDI_MAX_NOTES, &result);
    Midi_printResult(&result);   // prints stats over UART
    if (result.status == MIDI_OK)
        NotePlayer_playSequence(events, result.noteCount);
}

/* Testing Instructions:
Then from your PC: 
pip install pyserial
python send_midi.py twinkle.mid --port COM5
*/

// Paste your MIDI file bytes here — get them by running:
// python -c "d=open('song.mid','rb').read(); print(','.join(f'0x{b:02X}' for b in d))"
//static const uint8_t MY_SONG[] = { 0x4D,0x54,0x68,0x64, ... };

//Midi_parseBuffer(MY_SONG, sizeof(MY_SONG), events, MIDI_MAX_NOTES, &result);

// C major scale for testing purposes — replace with actual songs later.
NoteEvent scale[] = {
    {60, 400, 50},  /* C4 */
    {62, 400, 50},  /* D4 */
    {64, 400, 50},  /* E4 */
    {65, 400, 50},  /* F4 */
    {67, 400, 50},  /* G4 */
};

void test_playScale(void) {
    NotePlayer_playSequence(scale, 5);
}

// For testing: prints the key map over UART to confirm comms working and key map is correct.
void test_printKeyMap(void) {
    char buf[80];
    uint8_t i, j;
    for (i = 0; i < KEY_MAP_SIZE; i++) {
        const PianoKey* k = &KEY_MAP[i];
        for (j = 0; j < MAX_FINGER_OPTIONS; j++) {
            const FingerOption* o = &k->options[j];
            if (!o->valid) continue;
            snprintf(buf, sizeof(buf), "%s  finger=%s  pos=%.1fmm\r\n",
                k->name, fingerName(o->finger), o->motorPositionMM);
            HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), 100);
        }
    }
}

/* Private Defines */
#define SONG_PLAYER_HOMING_SPEED 0.2f
#define SONG_PLAYER_HOME_DIRECTION (-1.0f) /* TODO: replace with the direction that moves toward home. */
#define SONG_PLAYER_LIMIT_SWITCH_GPIO_Port GPIOA /* TODO: replace with the actual limit switch GPIO port. */
#define SONG_PLAYER_LIMIT_SWITCH_Pin GPIO_PIN_0 /* TODO: replace with the actual limit switch pin. */
#define SONG_PLAYER_LIMIT_SWITCH_ACTIVE_STATE GPIO_PIN_SET /* TODO: replace with the switch's active state. */

/* Private Function Declarations */
static bool song_player_isHomeSwitchActive(void);

/* Private Function Definitions */
/**
 * @brief Read the home limit switch state.
 * @return true when the home switch is active.
 */
static bool song_player_isHomeSwitchActive(void)
{
    return HAL_GPIO_ReadPin(SONG_PLAYER_LIMIT_SWITCH_GPIO_Port, SONG_PLAYER_LIMIT_SWITCH_Pin) == SONG_PLAYER_LIMIT_SWITCH_ACTIVE_STATE;
}


/* Public Function Definitions */
/**
 * @brief Run a blocking homing sequence until the home limit switch is reached.
 */
void song_player_homing(void)
{
    // Move hand towards home
    motor_control_setMotorSpeed(SONG_PLAYER_HOME_DIRECTION * SONG_PLAYER_HOMING_SPEED);

    // Wait until homing switch is pressed
    while (!song_player_isHomeSwitchActive())
    {
    }

    // Stop motor and set
    motor_control_setMotorSpeed(0.0f);
    motor_controller_encoderZeroPosition();
}
