
#ifndef P28_ACTIONSTATE_HPP_
#define P28_ACTIONSTATE_HPP_

#include "Drivebase.hpp"
#include "Robot.hpp"
#include "GameState.hpp"

namespace p28 {

/*
	How the ActionState should work

	1. It is meant as an intermediate representation of tasks, 
		a. It has no knowledge of how they work together to accomplish an objective (GameState's role)
		b. It has no knowledge of how to execute them (HardwareState's role)
	2. It generates a new version of itself given the current GameState and the current Robot state at each iteration
	3. It is handed down to generate the hardware state
*/

struct ActionState {
	bool openArm { false };
	bool releaseCup { false };

	DrivebasePath path;

	ActionState generate_next(Robot robState, GameState gmState) const;
};

} // !p28

#endif