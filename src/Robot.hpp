
#ifndef P28_ROBOTSTATE_HPP_
#define P28_ROBOTSTATE_HPP_

#include "Utils/Vec.hpp"
#include "Drivebase.hpp"
#include "sensors.hpp"
#include "GameState.hpp"

namespace p28 {

// Essentialy proprioception for the robot
struct Robot {
	float delta_s; // Delta time in second since last iteration
	unsigned long time_ms; // Time of the system
	
	Drivebase drvb;

	// Compute the next robot state from delta of the sensors and the game state
	Robot next(SensorState prevSensState, SensorState currSensState, 
				GameState prevGmState, GameState gmState) const;
};

} // !p28

#endif