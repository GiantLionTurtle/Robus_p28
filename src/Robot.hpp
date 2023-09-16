
#ifndef P28_ROBOT_HPP_
#define P28_ROBOT_HPP_

#include "Modules/Driver.hpp"
#include "Constants.hpp"
#include "Modules/Drivebase.hpp"
#include <Hardware/HardwareAccess.hpp>
#include <Utils/Pair.hpp>

namespace p28 {

/*
	@struct Robot_snapshot is a snapshot of the 
	robot at a point in time it contains the
	state of all actuators (Driver::State) in
	human/SI units as well as an estimate of
	the drivebase location
*/
struct Robot_snapshot {
	Drivebase::Desc location;
	Driver::State driverState;
};

/*
	@struct Robot_target is a target the robot
	is trying to reach. It is currently defined as

	* a drivebase location
	* a time of arrival

	as more modules are added, additionnal
	data is going to be added to the target
*/
struct Robot_target {
	Drivebase::Desc location;
	time_t time;
};

/*
	@struct Robot is the main robot abstraction
*/
struct Robot {
	Driver driver; // Driver to generate ActionStates from commands
	SensorState sensState; // Current state of all sensors

	Robot_snapshot previous; // Snapshot of the previous iteration
	Robot_snapshot current; // Snapshot of the current iteration
	Robot_target target; // Target the robot is trying to reach

	time_t time_ms; // Current time of the iteration
	float deltaTime_s; // Difference between previous and current iterations

	 // Returns a robot with an updated current and previous snapshot and sensState
	Robot snapshot() const;
	// Creates an ActionState and an updated robot
	Pair<ActionState, Robot> next_action() const;

private:
	// Create the drive command
	Driver::Command drive_command() const;
};


} // !p28

#endif