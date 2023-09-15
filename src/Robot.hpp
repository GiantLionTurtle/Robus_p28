
#ifndef P28_ROBOT_HPP_
#define P28_ROBOT_HPP_

#include "Modules/Driver.hpp"
#include "Constants.hpp"
#include "Modules/Drivebase.hpp"
#include <Hardware/HardwareAccess.hpp>
#include <Utils/Pair.hpp>

namespace p28 {

// Snapshot of the robot at a time
struct Robot_snapshot {
	Drivebase::Desc location;
	Driver::State driverState;
};

struct Robot_target {
	Drivebase::Desc location;
	time_t time;
};

struct Robot {
	Driver driver;
	SensorState sensState;

	Robot_snapshot previous;
	Robot_snapshot current;
	Robot_target target;

	time_t time_ms;
	float deltaTime_s; // Difference between previous and current

	 // Returns a robot with an updated current and previous snapshot and sensState
	Robot snapshot() const;
	Pair<ActionState, Robot> next_action() const;

	Driver::Command drive_command() const;
};


} // !p28

#endif