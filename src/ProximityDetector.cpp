#include <Arduino.h>
#include "LibRobus.h"
#include "Constants.hpp"

bool wall_detection()
{
    bool red = !digitalRead(14);
    bool green = !digitalRead(15);
    return (red && green);
}