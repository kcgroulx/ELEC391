/**
 * @file song_player.h
 * @brief Public interface for song playing
 */

#ifndef SRC_SONG_PLAYER_H_
#define SRC_SONG_PLAYER_H_

/* Includes */
#include <stdbool.h>
#include <stdint.h>

/* Defines */

/* Typedefs */
/**
 * @brief One scheduled note event with target position, finger selection, and timing.
 */
typedef struct {
   float targetPosition;
   uint8_t fingerBitmask;
   uint16_t pressDuration;
   uint16_t noteDuration;
} song_state_t;

/**
 * @brief Song playback sequence as an array of note events and progress counters.
 */
typedef struct {
    song_state_t* states;
    uint32_t num_states;
    uint32_t count;
} song_t;

/**
 * @brief High-level states for the piano bot control flow.
 */
typedef enum {
    INIT = 0,
    WAIT = 1,
    PLAY = 3,
    UHOH = 5
} piano_bot_state;

/* Public Function Declarations */
void song_player_init(song_t* song);
void song_player_step(void);
piano_bot_state song_player_getState(void);
bool song_player_homing(void);

#endif /* SRC_SONG_PLAYER_H_ */
