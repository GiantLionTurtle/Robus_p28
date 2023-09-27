
#ifndef P28_DRIVEBASE_HPP_
#define P28 DRIVEBASE_HPP_

#include <Arduino.h>

#include "PID.hpp"
#include "Constants.hpp"

namespace p28 {

float ticks_to_dist(uint32_t ticks);

} // !p28

#endif