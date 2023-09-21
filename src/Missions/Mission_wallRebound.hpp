
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
// template<int rightTurnDelay_ms = 500, int leftTurnDelay_ms = 500>
struct WallRebound {
	// Data specific to the mission
	bool turnRight_next { true };
	time_t turningStartTime { 0 }; // If start time is 0, it is assumed that the robot is not turning

	enum class TurnState { None, Left, Right };

	Epsilon eps{};

	void init() const {}

	Pair<Robot_target, WallRebound> update(Robot robot) const;

	// TurnState turnState() const
	// {
	// 	// if(turningStartTime == 0)
	// 	// 	return 
	// }
};

} // !Mission
} // !p28

#endif