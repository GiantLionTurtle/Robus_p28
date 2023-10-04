
#include <Arduino.h>
#include "LibRobus.h"
#include "Constants.hpp"
#include "ProximityDetector.hpp"

namespace p28 {

void init_detector()
{
    pinMode(GREEN_PIN, INPUT);
    pinMode(RED_PIN, INPUT);
}

bool wall_detection()
{
    bool red = !digitalRead(RED_PIN);
    bool green = !digitalRead(GREEN_PIN);
    return (red || green);
}

}