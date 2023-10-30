#ifndef P28_HARDWARESTATE_HPP_
#define P28_HARDWARESTATE_HPP_

#include "Utils/Vec2.hpp"
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

	// Function to mix the current state with a target 
	// hardware state with an exponential moving average
	// it helps smooth out motor output
	HardwareState mix(HardwareState hrdwState) const;
};

void set_hardwareState(struct HardwareState hwst);

HardwareState generate_hardwareState(Robot robot);

void printHarwareState (HardwareState state);

} // !p28

#endif