/**
 * @file song_player.h
 * @brief Public interface for song playing
 */

#ifndef SRC_SONG_PLAYER_H_
#define SRC_SONG_PLAYER_H_

/* Includes */
#include <stdbool.h>

/* Defines */

/* Typedefs */
typedef struct {
   float targetPosition;
   uint8_t fingerBitmask;
   uint16_t pressDuration;
   Uint16_t noteDurantion
} song_state_t;

typedef struct {
    song_state_t* states;
    uint32_t num_states;
    uint32_t count;
} song_t;

typedef enum {
    INIT = 0,
    HOME = 1,
    WAIT = 2,
    PLAY = 3,
    UHOH = 5
} piano_bot_state;

/* Public Function Declarations */
void song_player_homing(void);

#endif /* SRC_SONG_PLAYER_H_ */
