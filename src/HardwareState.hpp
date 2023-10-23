#ifndef P28_HARDWARESTATE_HPP_
#define P28_HARDWARESTATE_HPP_

#include "Utils/Vec.hpp"
#include "Robot.hpp"
#include "Constants.hpp"

/*
	How hardware state should work

	1. It is generated using an ActionState and a robot copy
		a. It translates the commanded actions into real actual commands such as motor values
		b. It updates the robot because it needs the feedback from the motors <== needs to change
	2. It is writen to the hardware using LibRobus core code
*/

namespace p28 {

struct HardwareState {
	p28::mt::Vec2 motors; // Values from [-1,1]
	int armAngle { kArm_closeAngle };   //angle of the arm 
	int cupAngle { kCup_closeAngle };   //angle of the servomotor that holds the cup
};

void set_hardwareState(struct HardwareState hwst);

HardwareState generate_hardwareState(Robot robot);

} // !p28

#endif