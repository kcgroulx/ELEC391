#ifndef PLANNED_SONG_PLAYER_H
#define PLANNED_SONG_PLAYER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float    position_mm;
    uint8_t  fingers_mask;
    uint16_t press_duration_ms;
    uint16_t travel_duration_ms;
} PlannedSongStep;

#define PLANNED_SONG_MAX_STEPS 256U

void PlannedSongPlayer_run(void);

#ifdef __cplusplus
}
#endif

#endif /* PLANNED_SONG_PLAYER_H */
