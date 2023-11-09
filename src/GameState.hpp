
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
namespace p28{

struct GameState {
	// Compute the next gamestate from sensor data deltas
	GameState generate_next(SensorState prevSensState, SensorState currSensState, DrivebaseState drvbState,Iteration_time it_time) const;
	static GameState initial(SensorState sensState);
};

} // !p28

#endif