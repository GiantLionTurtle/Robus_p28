
#ifndef P28_ROBOTSTATE_HPP_
#define P28_ROBOTSTATE_HPP_

#include "Utils/Vec.hpp"
#include "Drivebase.hpp"

namespace p28 {

// Essentialy proprioception for the robot
struct Robot {
	float delta_s; // Delta time in second since last iteration
	unsigned long millis; // Time of the system
	
	Drivebase drvb;
};

} // !p28

#endif