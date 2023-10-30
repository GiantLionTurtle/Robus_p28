
#ifndef P28_DRIVEBASE_HPP_
#define P28_DRIVEBASE_HPP_

#include <Arduino.h>
#include <LibRobus.h>

#include "PID.hpp"
#include "Constants.hpp"
#include "Utils/Vec2.hpp"

#include "Utils/Vector.hpp"
#include "Utils/Pair.hpp"
#include "Utils/Vec2.hpp"
#include "Iteration_time.hpp"
#include "Utils/Geometry.hpp"

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
	mt::Vec2 pos { 0.0, 0.0 }; // Position in m
	mt::Vec2 heading { 0.0, 1.0 }; // Normalised heading
	float angular_velocity { 0.0 };
	float trajectory_radius { 0.0 };

	mt::Vec2 wheelsVelocities { 0.0 }; // Velocity in m/s of each wheel

	// A point in time in ms, used to stop the path following as long as millis() < waitUntil
	unsigned long waitUntil { 0 }; 

	DrivebaseState() = default;
	DrivebaseState(mt::Vec2 pos_)
		: pos(pos_)
	{

	}

	// Returns the overall velocity of the drivebase
	float velocity() const;

	// Create a new drivebase state from encoder ticks alone
	DrivebaseState update_kinematics(mt::i32Vec2 prevEncTicks, mt::i32Vec2 currEncTicks, float delta_s) const;
	
	// Returns the drivebasestate if it were at the position
	// of the intersection between a line and the ray formed
	// by it's position and it's heading
	DrivebaseState intersect_line(mt::Line ln) const;

	mt::Vec2 get_motor_speed(mt::i32Vec2 prevEncTicks, mt::i32Vec2 currEncTicks, float delta_s) const;
};

struct Arc {
	mt::Vec2 tengeantStart;
	mt::Vec2 end;
	float radius;
	float length;

	void print() const;
};

// Action that the drivebase can do (high level)
struct PathSegment {
	mt::Vec2 targPos { 0.0 }; // Target position 
	mt::Vec2 targHeading { 0.0 };
	float targSpeed { 0.0 }; // Speed at the target position
	bool backward { false };

	PathSegment() = default;
	PathSegment(mt::Vec2 targPos_, mt::Vec2 targHeading_, float targSpeed_, bool backward);
};

// Arcs to follow and delays after it's done

struct DrivebasePath {
	// Vector<Pair<PathSegment, unsigned int>> path;
	
	Pair<PathSegment, unsigned int> segments[10];
	unsigned int index { 0 };
	unsigned int size { 0 };

	PathSegment current() const { return segments[index].first; }
};


struct Motor {
	PID pid;
	Error error;

	float hardware_output() const;
};

struct DrivebaseConcrete {
	Motor left;
	Motor right;

	PID headingPID;
	Error headingError;

	DrivebaseConcrete update(mt::Vec2 actualWheelVelocities, mt::Vec2 desiredWheelVelocities,
								mt::Vec2 currentHeading, mt::Vec2 targetHeading, Iteration_time it_time) const;
	mt::Vec2 hardware_output() const;
};

struct Drivebase {
	DrivebaseConcrete concrete;
	DrivebaseState state;
	DrivebasePath path;

	void update_path();
	void update_concrete(Iteration_time it_time);

	// Return new wheel velocities from the wheel velocities needed
	// to follow the arc + known heading error
	mt::Vec2 correct_heading(mt::Vec2 staged_wheelVelocities) const;
};

// Conversion for encoders to distance (meters)
float ticks_to_dist(int32_t ticks);
mt::Vec2 ticks_to_dist(mt::i32Vec2 bothTicks);
// Conversion distance (meters) to encoder ticks
int32_t dist_to_ticks(float dist);
mt::i32Vec2 dist_to_ticks(mt::Vec2 dist);

float velocity_for_point(float current_velocity, float target_velocity, float dist_to_target, float allowed_accel, float delta_s);
Arc arc_from_targetHeading(mt::Vec2 start, mt::Vec2 end, mt::Vec2 end_heading);
mt::Vec2 arcTurnToDest(Arc arc, float angularVelocity);

} // !p28

#endif