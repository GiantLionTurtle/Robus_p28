
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
	float speed { 0.0 };
	int32_t last_ticks { 0 };
	long int last_time_ms { 0 };
};
struct Drivebase {
	struct Motor left;
	struct Motor right;

	float x{ 0.75 }, y { 0.25 }; // Position in meters, from the bottom left corner of the field
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
float ticks_to_dist(int32_t ticks);
float accel_dist(float accel, float target_speed);

// Functions that move the robot must return 
// the part of the robot that has been moved
// with a modified state


struct Motor get_motor_speed(struct Motor motor, float delta_s);
struct Motor update_motor_at_speed(struct Motor motor, float speed, long int time_ms);


struct Drivebase zero_all(struct Drivebase drvb);
struct Drivebase set_motorTime(struct Drivebase drvb, long int time_ms);


// float velocity_profile(float target_vel, float dist_to_travel, float current_dist, float time_since_start);

}

#endif