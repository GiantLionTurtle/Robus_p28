
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
    float speed { 0.0 };
    int32_t last_ticks { 0.0 };
    long int last_time_ms { 0 };
};
struct Drivebase {
    Motor left;
    Motor right;

    float x, y;
    int direction; // FRONT, BACK, LEFT, RIGHT
};

// Conversion for encoders to distance (meters)
float ticks_to_dist(uint32_t ticks);


// Functions that move the robot must return 
// the part of the robot that has been moved
// with a modified state


Motor get_motor_speed(Motor motor, float delta_s);
Motor update_motor_at_speed(Motor motor, float speed, long int time_ms);

// Functions to move the robot 
// !!! RELATIVE TO IT'S OWN ORIENTATION !!!
// Negative distance means backward
Drivebase forward_dist(Drivebase drvb, float dist, float speed);
Drivebase forward_until_detect(Drivebase drvb, float dist, float speed, bool& detection);
Drivebase turn_right(Drivebase drvb);
Drivebase turn_left(Drivebase drvb);

// Move to a square, handle the orientation of the robot and such
// Simple, dumb strategie like go to x, turn, go to y
Drivebase move_to_square(Drivebase drvb, int square_x, int square_y);

} // !p28

#endif