
#ifndef P28_DRIVEBASE_HPP_
#define P28_DRIVEBASE_HPP_

#include <Arduino.h>
#include <LibRobus.h>

#include "PID.hpp"
#include "Constants.hpp"
#include "Utils/Vec2.hpp"

#include "Utils/Pair.hpp"
#include "Iteration_time.hpp"
#include "Utils/Geometry.hpp"
#include "SensorsState.hpp"
#include "Sensors/LineDetector.hpp"
#include <HardwareState.hpp>

/*
	How the drivebase should work:

	1. PathCheckPoints are fed one at a time (end point, end heading, end velocity)
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

struct Arc {
	mt::Vec2 tengeantStart;
	mt::Vec2 end;
	float radius;
	float length;

	void print() const;
};

// Action that the drivebase can do (high level)
struct PathCheckPoint {
	mt::Vec2 targPos { 0.0 }; // Target position 
	mt::Vec2 targHeading { 0.0 };
	float targVel { 0.0 }; // Speed at the target position
	float maxVel { kMaxVel };
	unsigned int delay_before { 0 };
	bool turn_only { false };
	bool backward { false };

	PathCheckPoint() = default;
	PathCheckPoint(mt::Vec2 targPos_, mt::Vec2 targHeading_, 
					float targVel_ = 0.0, bool backward_ = false, float maxVel_ = kMaxVel, unsigned int delay_before_ = 0);

	static PathCheckPoint make_turn(mt::Vec2 targHeading_, unsigned int delay_before = 0);
};

// Arcs to follow and delays after it's done

struct DrivebasePath {
	PathCheckPoint segments[kMaxCheckPointForPath];
	
	unsigned int index { 0 };
	unsigned int size { 0 };

	PathCheckPoint& current() { return segments[index]; }
	void add_checkPoint(PathCheckPoint segment);
	bool finished() const { return index >= size; }
};


struct Motor {
	PID pid;
	Error error;

	float hardware_output() const;
};

struct Drivebase {
	mt::Vec2 pos { 0.0, 0.0 }; // Position in m
	mt::Vec2 heading { 0.0, 1.0 }; // Normalised heading
	float angular_velocity { 0.0 };
	float trajectory_radius { 0.0 };

	mt::Vec2 wheelsVelocities { 0.0 }; // Velocity in m/s of each wheel

	Motor leftWheel;
	Motor rightWheel;

	PID headingPID;
	Error headingError;

	time_t waitUntil_ms { 0 };

	DrivebasePath path;
	bool followLine{false};

	void update(SensorState currentSensState, SensorState prevSensState, Iteration_time it_time);
	void update_followLine(SensorState currentSensState, SensorState prevSensState, Iteration_time it_time);
	void update_path(Iteration_time it_time);
	void set_path(DrivebasePath path_, Iteration_time it_time);

	float velocity();
	
	void update_kinematics(mt::i32Vec2 prevEncTicks, mt::i32Vec2 currEncTicks, float delta_s);

	HardwareState aggregate(HardwareState hrdwState);

private:
	void update_wheels(mt::Vec2 target_wheelVels, double delta_s);
	void update_wheels(mt::Vec2 target_wheelVels, mt::Vec2 target_heading, double delta_s);

	void update_follow_arc(PathCheckPoint follow, Iteration_time it_time);
	void update_turn(PathCheckPoint follow, Iteration_time it_time);

	// Returns an offset to wheel velocities to correct 
	// the heading
	mt::Vec2 correct_heading() const;
};

// Conversion for encoders to distance (meters)
float ticks_to_dist(int32_t ticks);
mt::Vec2 ticks_to_dist(mt::i32Vec2 bothTicks);
// Conversion distance (meters) to encoder ticks
int32_t dist_to_ticks(float dist);
mt::i32Vec2 dist_to_ticks(mt::Vec2 dist);

float velocity_for_point(float current_vel, float target_vel, float max_vel, float target_dist, float accel, float delta_s);
Arc arc_from_targetHeading(mt::Vec2 start, mt::Vec2 end, mt::Vec2 end_heading);
mt::Vec2 arcTurnToDest(Arc arc, float angular_vel);


} // !p28

#endif