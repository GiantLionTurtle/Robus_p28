
#ifndef P28_MISSION_WALLREBOUND_HPP_
#define P28_MISSION_WALLREBOUND_HPP_

#include <Robot.hpp>
#include "Mission.hpp"

#include <Utils/Pair.hpp>

namespace p28 {
namespace Mission {

/*
	@struct Mission_wallRebound is a demo mission
	where the robot goes forward until it hits a
	wall, then alternates between turning right
	and left for a specified amount of time
*/
struct WallRebound {
	// Data specific to the mission
	int turnRightAmount_s { 1 };
	int turnLeftAmount_s { 2 };
	bool turnRight_next { true };
	time_t turningStartTime { 0 }; // If start time is 0, it is assumed that the robot is not turning

	Epsilon eps{};

	void init() const {}

	Pair<Robot_target, WallRebound> update(Robot robot) const;
};

} // !Mission
} // !p28

#endif