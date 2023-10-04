
#include <Arduino.h>
#include "LibRobus.h"
#include "Constants.hpp"
#include "WhistleDetector.hpp"

namespace p28 {

void init_whistle()
{
    pinMode(WHISTLE_PIN, INPUT);
}

bool whistle_detection()
{
    bool whistle = digitalRead(WHISTLE_PIN);
    return whistle;
}

}
