
#ifndef P28_DRIVEBASE_HPP_
#define P28_DRIVEBASE_HPP_

#include <Arduino.h>
#include <LibRobus.h>

#include "PID.hpp"
#include "Constants.hpp"
#include "Utils/Vec.hpp"

#include "Utils/Vector.hpp"
#include "Utils/Pair.hpp"
#include "Utils/Vec.hpp"

namespace p28 {

struct DrivebaseState {
	mt::Vec2 pos; // Position in m
    mt::Vec2 heading; // Heading with a length of velocity (m/s)

    mt::Vec2 wheelsVelocities; // Velocity in m/s of each wheel

	// A point in time in ms, used to stop the path following as long as millis() < waitUntil
	unsigned long waitUntil { 0 }; 

	// Create a new drivebase state from encoder ticks alone
	DrivebaseState update(mt::i32Vec2 prevEncTicks, mt::i32Vec2 currEncTicks, float delta_s) const;
};

// Action that the drivebase can do (high level)
struct PathSegment {
	p28::mt::Vec2 targPos { 0.0 }; // Target position 
	float targSpeed { 0.0 }; // Speed at the target position

	 // Radius of the arc path
	 // + -> anticlockwise
	 // - -> clockwise
	float pathRadius { 0.0 };

	PathSegment() = default;
	PathSegment(DrivebaseState drvbState, p28::mt::Vec2 targPos_, float targSpeed_, p28::mt::Vec2 targHeading_);
	PathSegment(p28::mt::Vec2 targPos_, float targSpeed_);
	PathSegment(p28::mt::Vec2 targPos_, float targSpeed_, float pathRadius_);
};

// Arcs to follow and delays after it's done

struct DrivebasePath {
    Vector<Pair<PathSegment, unsigned int>> path;
    unsigned int index { 0 };

	PathSegment current() const { return path[index].first; }

	DrivebasePath update_path(DrivebaseState drvbState) const;
};


struct Motor {
	int ID;
	PID pid;
	Error error;
	int32_t last_ticks { 0 };
};

struct Drivebase {
	Motor left;
	Motor right;
	DrivebaseState state;

	Pair<mt::Vec2, Drivebase> hardware_output(PathSegment const& follow, unsigned long time_ms, float delta_s) const;
};

// Conversion for encoders to distance (meters)
float ticks_to_dist(int32_t ticks);
mt::Vec2 ticks_to_dist(mt::i32Vec2 bothTicks);
float accel_dist(float accel, float target_speed);


// Functions that move the robot must return 
// the part of the robot that has been moved
// with a modified state
struct Motor get_motor_speed(struct Motor motor, float delta_s);
mt::Vec2 get_motor_speed(mt::i32Vec2 prevEncTicks, mt::i32Vec2 currEncTicks, float delta_s);
struct Motor update_motor_at_speed(struct Motor motor, float speed, long int time_ms);


struct Drivebase zero_all(struct Drivebase drvb);
struct Drivebase set_motorTime(struct Drivebase drvb, long int time_ms);



// float velocity_profile(float target_vel, float dist_to_travel, float current_dist, float time_since_start);

} // !p28

#endif