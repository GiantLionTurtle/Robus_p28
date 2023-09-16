
#ifndef P28_HARDWAREACCESS_HPP_
#define P28_HARDWAREACCESS_HPP_

#include <Utils/Vec.hpp>

/*
    Here are defined hardware call through LibRobUS
*/

namespace p28 {

/*
	@struct ActionState contains all the output
	state of the system, it is intended to be computed
	every loop cycle and be writen to the hardware
	subsequently. The values contained in it are meant
	to be written directly on hardware, these are not
	SI units
*/
struct ActionState {
	// Values go from [-1, 1], not to be confused with the motordrivers
	mt::Vec2 driveBaseMotor; 
};
/*
	@struct SensorState contains the sensor state
	of the system, it is intended to be updated every
	loop cycle
*/
struct SensorState {
	mt::iVec2 driveEncoders; // Units are encoder ticks

	bool bumperSwitches[4]; // Could be combined into one int
};

void init();
SensorState readSensors();
void writeActions(ActionState const& act);

}

#endif