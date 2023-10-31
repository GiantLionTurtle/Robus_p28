
#ifndef P28_GAMESTATE_HPP_
#define P28_GAMESTATE_HPP_

#include "Utils/Pair.hpp"
#include "sensors.hpp"
#include "Drivebase.hpp"
#include "Iteration_time.hpp"

/*
	How GameState should work

	1. GameState is initialized, each mission is marked as todo if it must be done
	2. GameState creates an updated copy of itself using the sensors's information
		a. It digest sensors information into game langage (lane, zone, etc.)
		b. It decides the stat of each objective (underway, done, todo, starting)
		c. It decides if the program should terminate
	3. Specific actions to accomplish the objectives are outside the scope of this structure
		-> see ActionState.hpp
*/

namespace p28 {

struct Objective {
	enum Doneness : char { Todo, Start, UnderWay, Done };

	Doneness donneness;
	time_t start_ms;

	bool todo() const { return donneness == Doneness::Todo; }
	bool start() const { return donneness == Doneness::Start; }
	bool underway() const { return donneness == Doneness::UnderWay; }
	bool done() const { return donneness == Doneness::Done; }

	Objective advance(bool start_cond, bool end_cond, time_t time_ms) const
	{
		if(todo() && start_cond) {
			return Objective { .donneness=Doneness::Start, .start_ms=time_ms };
		}
		if(start()) {
			return Objective { .donneness=Doneness::UnderWay, .start_ms=start_ms };
		}
		if(underway() && end_cond) {
			return Objective { .donneness=Doneness::Done, .start_ms=start_ms };
		}
		return *this;
	}
};


struct MissionState {

	// Mission objectives if the robot is not in race mode
	Objective knock_cup { Objective::Todo };
	Objective trap_ball { Objective::Todo };
	Objective one_turn { Objective::Todo };
	Objective one_shortcut_turn { Objective::Todo };
	Objective test { Objective::Done };
};

struct GameState {
	int zone { 0 }; // The zone the robot is currently in
	int lane { 0 }; // 0->3, 0 being closest to center
	bool over { false };
	int target_lane { -1 }; // Lane to follow

	// How the mission is going (if not in race mode)
	MissionState missions;

	// Compute the next gamestate from sensor data deltas
	GameState generate_next(SensorState prevSensState, SensorState currSensState, DrivebaseState drvbState,Iteration_time it_time) const;
	static GameState initial(SensorState sensState);
};

} // !p28

#endif