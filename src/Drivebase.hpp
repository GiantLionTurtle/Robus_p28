
#ifndef P28_DRIVEBASE_HPP_
#define P28_DRIVEBASE_HPP_

#include <Arduino.h>
#include <LibRobus.h>

#include "PID.hpp"
#include "Constants.hpp"
#include "Utils/Vec.hpp"

namespace p28 {

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
struct DrivebaseState {
    p28::mt::Vec2 pos; // Position in m
    p28::mt::Vec2 heading; // Heading with a length of velocity (m/s)

    p28::mt::Vec2 wheelsVelocities; // Velocity in m/s of each wheel
};

// Action that the drivebase can do (high level)
struct DrivebaseActionState {
	p28::mt::Vec2 targPos { 0.0 }; // Target position 
	float targSpeed { 0.0 }; // Speed at the target position

	 // Radius of the arc path
	 // + -> anticlockwise
	 // - -> clockwise
	float pathRadius { 0.0 };

	DrivebaseActionState() = default;
	DrivebaseActionState(DrivebaseState drvbState, p28::mt::Vec2 targPos_, float targSpeed_, p28::mt::Vec2 targHeading_);
	DrivebaseActionState(p28::mt::Vec2 targPos_, float targSpeed_);
	DrivebaseActionState(p28::mt::Vec2 targPos_, float targSpeed_, float pathRadius_);
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

}

#endif