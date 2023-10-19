
#ifndef P28_ACTIONSTATE_HPP_
#define P28_ACTIONSTATE_HPP_

#include "Drivebase.hpp"
#include "Robot.hpp"
#include "GameState.hpp"

namespace p28 {

// Somewhat abstract actions the robot can take
struct ActionState {
	bool openArm { false };
	bool releaseCup { false };

	DrivebasePath path;
};

ActionState generate_actionState(ActionState prevActState, Robot robState, GameState gmState);

} // !p28

#endif