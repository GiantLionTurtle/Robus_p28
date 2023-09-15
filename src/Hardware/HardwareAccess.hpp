
#ifndef P28_HARDWAREACCESS_HPP_
#define P28_HARDWAREACCESS_HPP_

#include <Utils/Vec.hpp>

/*
    Here are defined hardware call through LibRobUS
*/

namespace p28 {

/*
	@struct RobotState_out contains all the output
	state of the system, it is intended to be computed
	every loop cycle and be writen to the hardware
	subsequently
*/
struct ActionState {
	// Values go from [-1, 1], not to be confused with the motordrivers
	float driveBaseMotor[2]; 
};
/*
	@struct RobotState_in contains the sensor state
	of the system, it is intended to be updated every
	loop cycle
*/
struct SensorState {
	mt::iVec2 driveEncoders;

	bool bumperSwitches[4]; // Could be combined into one int
};

void init();
SensorState readSensors();
void writeActions(ActionState const& act);

}

#endif