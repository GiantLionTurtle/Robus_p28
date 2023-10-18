
#ifndef P28_GAMESTATE_HPP_
#define P28_GAMESTATE_HPP_

#include "Utils/Pair.hpp"
#include "sensors.hpp"

namespace p28 {

enum class Objective : char { Todo, Start, UnderWay, Done };

struct MissionState {

	// Mission objectives if the robot is not in race mode
	Objective knock_cup { Objective::Todo };
	Objective trap_ball { Objective::Todo };
	Objective one_cw_turn { Objective::Todo };
	Objective one_cw_shortcut_turn { Objective::Todo };
};

struct GameState {
	int zone { 0 }; // The zone the robot is currently in
	int lane { 0 }; // 0->3, 0 being closest to center
	bool over { false };

	// How the mission is going (if not in race mode)
	MissionState missionState;

	// Compute the next gamestate from sensor data deltas
	GameState next(SensorState prevSensState, SensorState currSensState) const;
};

} // !p28

#endif