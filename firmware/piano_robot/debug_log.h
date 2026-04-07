#ifndef PIANOBOT_DEBUG_LOG_H
#define PIANOBOT_DEBUG_LOG_H

#define PIANO_DEBUG_LOG_ENABLED 1

#if PIANO_DEBUG_LOG_ENABLED
#include "Arduino.h"
#define DEBUG_PRINT(...) Serial.print(__VA_ARGS__)
#define DEBUG_PRINTLN(...) Serial.println(__VA_ARGS__)
#else
#define DEBUG_PRINT(...) do { } while (0)
#define DEBUG_PRINTLN(...) do { } while (0)
#endif

#endif /* PIANOBOT_DEBUG_LOG_H */
