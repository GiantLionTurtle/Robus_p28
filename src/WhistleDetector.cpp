#include <Arduino.h>
#include "LibRobus.h"
#include "Constants.hpp"

bool wall_detection()
{
    bool whistle = digitalRead(18);
    return whistle;
}