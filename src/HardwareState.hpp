#ifndef P28_HARDWARESTATE_HPP_
#define P28_HARDWARESTATE_HPP_

#include "Utils/Vec2.hpp"
#include "Constants.hpp"
#include "Iteration_time.hpp"

/*
	How hardware state should work

	1. It is generated using an ActionState and a robot copy
		a. It translates the commanded actions into real actual commands such as motor values
		b. It updates the robot because it needs the feedback from the motors <== needs to change
	2. It is writen to the hardware using LibRobus core code
*/

namespace p28 {

struct HardwareState {
	p28::mt::Vec2 motors {0, 0}; // Values from [-1,1]

	int clawAngle { kClaw_openAngle};
	int armAngle { kArm_restAngle};

	int conveyorSteps { 0 };
	int trapAngle { kBinTrap_closedAngle};    // angle for closed trap initial position
	// angles of the servomotors of the color selected in the bin 
	int bin_select_angles [3] { kBinSelect_closedAngle, kBinSelect_closedAngle, kBinSelect_closedAngle};	
	// Function to mix the current state with a target 
	// hardware state with an exponential moving average
	// it helps smooth out motor output
	HardwareState mix(HardwareState hrdwState) const;

	static HardwareState initial();
};

void apply_hardwareState(HardwareState hwst, Iteration_time it_time);

void print (HardwareState state);

} // !p28

#endif