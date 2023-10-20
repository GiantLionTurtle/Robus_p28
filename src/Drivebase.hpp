
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

/*
	How the drivebase should work:

	1. PathSegments are fed one at a time (end point, end heading, end velocity)
	2. The drivebase computes an arc to complete the segment
	3. The drivebase tries to follow said arcs using
		a. Forward kinematics
		b. Acceleration motion profiles
		c. Heading pid
	4. The drivebase uses inverse kinematics to understand it's position
	5. The new position is agremented with other sensors outside the drivebase code's scope
	6. The new position is used in step 2
*/

namespace p28 {

struct DrivebaseState {
	mt::Vec2 pos; // Position in m
    mt::Vec2 heading; // Normalised heading
	float velocity;

    mt::Vec2 wheelsVelocities; // Velocity in m/s of each wheel

	// A point in time in ms, used to stop the path following as long as millis() < waitUntil
	unsigned long waitUntil { 0 }; 

	// Create a new drivebase state from encoder ticks alone
	DrivebaseState update(mt::i32Vec2 prevEncTicks, mt::i32Vec2 currEncTicks, float delta_s) const;
};


struct Arc {
	mt::Vec2 tengeantStart;
	mt::Vec2 end;
	float radius;
	float length;
};
struct Line {
	mt::Vec2 origin;
	mt::Vec2 dir;

	mt::Vec2 line_intersection(Line const& l2) const;
};

// Action that the drivebase can do (high level)
struct PathSegment {
	mt::Vec2 targPos { 0.0 }; // Target position 
	mt::Vec2 targHeading { 0.0 };
	float targSpeed { 0.0 }; // Speed at the target position

	PathSegment() = default;
	PathSegment(mt::Vec2 targPos_, mt::Vec2 targHeading_, float targSpeed_);
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


mt::Vec2 get_motor_speed(mt::i32Vec2 prevEncTicks, mt::i32Vec2 currEncTicks, float delta_s);
Pair<Motor, float> update_motor_at_speed(Motor motor, float set_speed,float actual_speed, float delta_s);

float velocity_for_point(float current_velocity, float target_velocity, float dist_to_target, float allowed_accel);
Arc arc_from_targetHeading(mt::Vec2 start, mt::Vec2 end, mt::Vec2 end_heading);
mt::Vec2 arcTurnToDest(Arc arc, float angularVelocity);

// float velocity_profile(float target_vel, float dist_to_travel, float current_dist, float time_since_start);

} // !p28

#endif