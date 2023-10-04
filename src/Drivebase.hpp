
#ifndef P28_DRIVEBASE_HPP_
#define P28_DRIVEBASE_HPP_

#include <Arduino.h>
#include <LibRobus.h>

#include "PID.hpp"
#include "Constants.hpp"

namespace p28 {

struct Motor {
    int ID;
    PID pid;
    Error error;
    float speed { 0.0 };
    int32_t last_ticks { 0 };
    long int last_time_ms { 0 };
};
struct Drivebase {
    Motor left;
    Motor right;

    float x{ 0.75 }, y { 0.25 }; // Position in meters, from the bottom left corner of the field
    int orientation { FRONT }; // FRONT, BACK, LEFT, RIGHT
};

// Conversion for encoders to distance (meters)
float ticks_to_dist(int32_t ticks);


// Functions that move the robot must return 
// the part of the robot that has been moved
// with a modified state


Motor get_motor_speed(Motor motor, float delta_s);
Motor update_motor_at_speed(Motor motor, float speed, long int time_ms);

// Functions to move the robot 
// !!! RELATIVE TO IT'S OWN ORIENTATION !!!
// Negative distance means backward
Drivebase forward_dist(Drivebase drvb, float dist, float speed);
Drivebase forward_until_detect(Drivebase drvb, float dist, float speed, float& traveled_dist, bool& detection);
Drivebase turn_right(Drivebase drvb);
Drivebase turn_left(Drivebase drvb);

// Moves the drivebase by increments of squares in one of 
// 4 directions (LEFT, RIGHT, FRONT, REAR)
// If the drivebase does not start at the center of a square,
// it still gets to the center of the destination square with
// respect to it's move direction
Drivebase move_to_square(Drivebase drvb, int direction, int n_squares);
// Tries to move 1 square in a direction, but tries to detect a wall 
// and moves back to the starting position if there was a wall
Drivebase move_to_square_or_detect(Drivebase drvb, int direction, bool& detection);
Drivebase orient_toward_direction(Drivebase drvb, int direction);

Drivebase zero_all(Drivebase drvb);
Drivebase set_motorTime(Drivebase drvb, long int time_ms);


// float velocity_profile(float target_vel, float dist_to_travel, float current_dist, float time_since_start);

} // !p28

#endif