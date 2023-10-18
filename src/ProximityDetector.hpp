
#ifndef P28_PROXIMITYDETECTOR_HPP_
#define P28_PROXIMITYDETECTOR_HPP_

#include <Arduino.h>
#include "LibRobus.h"
#include "Constants.hpp"

#define GREEN_PIN 46
#define RED_PIN 47

namespace p28 {

void init_detector();

bool wall_detection();

} // !p28

#endif