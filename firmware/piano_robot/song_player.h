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

void SongPlayer_run(void);            /* receive MIDI over Serial and play it    */
void song_player_homing(void);        /* blocking homing sequence                */
void test_playScale(void);            /* play C major scale for hardware testing */
void test_printKeyMap(void);          /* print key map over Serial               */
void test_fingersSolenoidOnly(void);  /* fire each solenoid, no motor movement   */
void test_allFingers(void);           /* move + press sequence hitting all 5     */
void test_limitSwitchDebug(void);     /* print limit switch state for debugging  */
void test_limitSwitchThenPlay(void);  /* home via limit switch, then play scale  */
void test_randomKeys(void);           /* random notes using all fingers          */
void test_CmajorScale(void);          /* C major scale C2–C3                    */
void test_happyBirthday(void);        /* Happy Birthday in C major              */
void test_solenoidPairs(void);        /* fire all 2-finger combos simultaneously */

#ifdef __cplusplus
}
#endif

#endif /* SONG_PLAYER_H */
