#include <Arduino.h>

#include "config.h"
#include "pins.h"

enum class State {
    CALIBRATE,
    EXPLORE_A,
    CROSS_BRIDGE,
    EXPLORE_B,
    RETURN_TO_START,
    FAST_RUN,
    FINISHED,
};

static State state = State::CALIBRATE;

void setup()
{
    Serial.begin(SERIAL_BAUD);
    pinMode(STATUS_LED, OUTPUT);
}

void loop()
{
    switch (state) {
    case State::CALIBRATE:
        break;
    case State::EXPLORE_A:
        break;
    case State::CROSS_BRIDGE:
        break;
    case State::EXPLORE_B:
        break;
    case State::RETURN_TO_START:
        break;
    case State::FAST_RUN:
        break;
    case State::FINISHED:
        break;
    }
}
