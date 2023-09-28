
#ifndef P28_DRIVEBASE_HPP_
#define P28 DRIVEBASE_HPP_

#include <Arduino.h>

#include "PID.hpp"
#include "Constants.hpp"

namespace p28 {

struct Motor {
    int ID;
    PID pid;
    Error error;
    float speed;
    int32_t last_ticks;
    long int last_time_ms;
};

float ticks_to_dist(uint32_t ticks);

Motor get_motor_speed(Motor motor, float delta_s);
Motor update_motor_at_speed(Motor motor, float speed, long int time_ms);

} // !p28

#endif