#include <Arduino.h>

#include "config.h"

namespace
{
constexpr size_t testFingerIndex = 0U;
constexpr int testFingerPin = app_config::finger_pins[testFingerIndex];
constexpr uint32_t togglePeriodMs = 1000U;

bool fingerOn = true;
uint32_t lastToggleMs = 0U;
}

void setup()
{
    Serial.begin(115200);
    pinMode(testFingerPin, OUTPUT);
    digitalWrite(testFingerPin, app_config::finger_active_level);
    lastToggleMs = millis();

    Serial.print("finger=");
    Serial.print(testFingerIndex);
    Serial.print(" pin=");
    Serial.print(testFingerPin);
    Serial.print(" state=");
    Serial.println(app_config::finger_active_level == HIGH ? "HIGH" : "LOW");
}

void loop()
{
    const uint32_t nowMs = millis();
    if ((nowMs - lastToggleMs) >= togglePeriodMs)
    {
        lastToggleMs = nowMs;
        fingerOn = !fingerOn;
        digitalWrite(testFingerPin, fingerOn ? app_config::finger_active_level : !app_config::finger_active_level);

        Serial.print("finger=");
        Serial.print(testFingerIndex);
        Serial.print(" state=");
        Serial.println(fingerOn ? "ON" : "OFF");
    }
}
