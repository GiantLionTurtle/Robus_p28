
#ifndef P28_ACTIONSTATE_HPP_
#define P28_ACTIONSTATE_HPP_

#include "Drivebase.hpp"

namespace p28 {

// Somewhat abstract actions the robot can take
struct ActionState {
	bool openArm { false };
	bool releaseCup { false };

	DrivebaseActionState drvbActionState;
};

} // !p28

#endif