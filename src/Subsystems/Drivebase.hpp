
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
#include <Paths.hpp>

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
	
	Paths::Path path;
	bool followLine{false};

	void update(SensorState currentSensState, SensorState prevSensState, Iteration_time it_time);
	void update_followLine(SensorState currentSensState, SensorState prevSensState, Iteration_time it_time);
	void update_path(Iteration_time it_time);
	void set_path(Paths::Path path_, Iteration_time it_time);

	float velocity();
	
	void update_kinematics(mt::i32Vec2 prevEncTicks, mt::i32Vec2 currEncTicks, float delta_s);

	HardwareState aggregate(HardwareState hrdwState);

private:
	void update_wheels(mt::Vec2 target_wheelVels, double delta_s);
	void update_wheels(mt::Vec2 target_wheelVels, mt::Vec2 target_heading, double delta_s);

	void update_follow_arc(Paths::CheckPoint follow, Iteration_time it_time);
	void update_turn(Paths::CheckPoint follow, Iteration_time it_time);

	// Returns an offset to wheel velocities to correct 
	// the heading
	mt::Vec2 correct_heading() const;

	mt::Vec2 arc_to_motorVels(Paths::Arc arc, float angular_vel);
};

// Conversion for encoders to distance (meters)
float ticks_to_dist(int32_t ticks);
mt::Vec2 ticks_to_dist(mt::i32Vec2 bothTicks);
// Conversion distance (meters) to encoder ticks
int32_t dist_to_ticks(float dist);
mt::i32Vec2 dist_to_ticks(mt::Vec2 dist);

float velocity_for_point(float current_vel, float target_vel, float max_vel, float target_dist, float accel, float delta_s);

} // !p28

#endif