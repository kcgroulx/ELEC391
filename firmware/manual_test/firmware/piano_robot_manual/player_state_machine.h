#ifndef PIANOBOT_PLAYER_STATE_MACHINE_H
#define PIANOBOT_PLAYER_STATE_MACHINE_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    float targetPosition;
    uint8_t fingerBitmask;
    uint16_t pressDuration;
    uint16_t noteDuration;
} song_state_t;

typedef struct
{
    song_state_t* states;
    uint32_t num_states;
    uint32_t count;
} song_t;

typedef enum
{
    PLAYER_STATE_INIT = 0,
    PLAYER_STATE_WAIT = 1,
    PLAYER_STATE_PLAY = 3,
    PLAYER_STATE_FAULT = 5
} player_state_t;

void player_state_machine_init(song_t* song);
void player_state_machine_step(void);
player_state_t player_state_machine_get_state(void);
bool player_state_machine_home(void);

#endif
