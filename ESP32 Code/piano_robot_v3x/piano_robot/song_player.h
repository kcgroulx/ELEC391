#ifndef SONG_PLAYER_H
#define SONG_PLAYER_H

/*
 * song_player.h
 * ===========================================================================
 * Top-level song playing logic and test helpers.
 * ===========================================================================
 */

#ifdef __cplusplus
extern "C" {
#endif

void SongPlayer_run(void);       /* receive MIDI over Serial and play it    */
void song_player_homing(void);   /* blocking homing sequence                */
void test_playScale(void);       /* play C major scale for hardware testing */
void test_printKeyMap(void);     /* print key map over Serial               */

#ifdef __cplusplus
}
#endif

#endif /* SONG_PLAYER_H */
