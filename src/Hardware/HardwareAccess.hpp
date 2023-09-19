
#ifndef P28_HARDWAREACCESS_HPP_
#define P28_HARDWAREACCESS_HPP_

#define DEBUG_MODE

#include <Utils/Vec.hpp>
#include <Constants.hpp>

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
	mt::Vec2 driveBaseMotor { 0.0 }; 
};
/*
	@struct SensorState contains the sensor state
	of the system, it is intended to be updated every
	loop cycle
*/
struct SensorState {
	time_t time_ms;

	mt::iVec2 driveEncoders; // Units are encoder ticks

	int8_t bumperSwitches[4]; // Could be combined into one int

	void print() const;
};

// Returns the diff of two action states
SensorState operator-(SensorState const& lhs, SensorState rhs);

void init();
SensorState readSensors();
void writeActions(ActionState const& act);

} // !p28

#endif