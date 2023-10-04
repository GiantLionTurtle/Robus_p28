
#ifndef P28_DRIVEBASE_HPP_
#define P28_DRIVEBASE_HPP_

#include <Arduino.h>
#include <LibRobus.h>

#include "PID.hpp"
#include "Constants.hpp"



struct Motor {
	int ID;
	struct PID pid;
	struct Error error;
	float speed { 0.0 };
	int32_t last_ticks { 0 };
	long int last_time_ms { 0 };
};
struct Drivebase {
	struct Motor left;
	struct Motor right;

	int sq_x { 1 }, sq_y { 0 }; // Position in squares
	float x{ 0.75 }, y { 0.25 }; // Position in meters, from the bottom left corner of the field
	int orientation { FRONT }; // FRONT, BACK, LEFT, RIGHT
};

// Conversion for encoders to distance (meters)
float ticks_to_dist(int32_t ticks);


// Functions that move the robot must return 
// the part of the robot that has been moved
// with a modified state


struct Motor get_motor_speed(struct Motor motor, float delta_s);
struct Motor update_motor_at_speed(struct Motor motor, float speed, long int time_ms);

// Functions to move the robot 
// !!! RELATIVE TO IT'S OWN ORIENTATION !!!
// Negative distance means backward
struct Drivebase forward_dist(struct Drivebase drvb, float dist, float speed);
struct Drivebase forward_until_detect(struct Drivebase drvb, float dist, float speed, float& traveled_dist, bool& detection);
struct Drivebase turn_right(struct Drivebase drvb);
struct Drivebase turn_left(struct Drivebase drvb);

// Moves the drivebase by increments of squares in one of 
// 4 directions (LEFT, RIGHT, FRONT, REAR)
// If the drivebase does not start at the center of a square,
// it still gets to the center of the destination square with
// respect to it's move direction
struct Drivebase move_to_square(struct Drivebase drvb, int direction, int n_squares);
// Tries to move 1 square in a direction, but tries to detect a wall 
// and moves back to the starting position if there was a wall
struct Drivebase move_to_square_or_detect(struct Drivebase drvb, int direction, bool& detection);
struct Drivebase orient_toward_direction(struct Drivebase drvb, int direction);

struct Drivebase zero_all(struct Drivebase drvb);
struct Drivebase set_motorTime(struct Drivebase drvb, long int time_ms);


// float velocity_profile(float target_vel, float dist_to_travel, float current_dist, float time_since_start);



#endif