
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
	double speed { 0.0 };
	int32_t last_ticks { 0 };
	long int last_time_ms { 0 };
};
struct Drivebase {
	struct Motor left;
	struct Motor right;

	double x{ 0.75 }, y { 0.25 }; // Position in meters, from the bottom left corner of the field
};

// Conversion for encoders to distance (meters)
double ticks_to_dist(int32_t ticks);
double accel_dist(double accel, double target_speed);

// Functions that move the robot must return 
// the part of the robot that has been moved
// with a modified state


struct Motor get_motor_speed(struct Motor motor, double delta_s);
struct Motor update_motor_at_speed(struct Motor motor, double speed, long int time_ms);


struct Drivebase zero_all(struct Drivebase drvb);
struct Drivebase set_motorTime(struct Drivebase drvb, long int time_ms);


// double velocity_profile(double target_vel, double dist_to_travel, double current_dist, double time_since_start);



#endif