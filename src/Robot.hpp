
#ifndef P28_ROBOTSTATE_HPP_
#define P28_ROBOTSTATE_HPP_

#include "Utils/Vec2.hpp"
#include "Drivebase.hpp"
#include "sensors.hpp"
#include "GameState.hpp"
#include "Iteration_time.hpp"

/*
	How the Robot should work

	It contains a representation of all subsistems.
	
	1. It generates a new copy of itself using sensor data, gamestate data and time
*/

namespace p28 {

// Essentialy proprioception for the robot
struct Robot {
	bool openArm { false };
	bool releaseCup { false };

	Drivebase drvb;

	// Compute the next robot state from delta of the sensors and the game state
	void generate_next(	SensorState prevSensState, SensorState currSensState, 
								GameState prevGmState, GameState gmState, Iteration_time it_time);
};

 Drivebase follow_line(Drivebase drvb);

} // !p28

#endif