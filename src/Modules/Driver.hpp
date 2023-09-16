
#ifndef P28_DRIVER_HPP_
#define P28_DRIVER_HPP_

#include "Hardware/MotorDriver.hpp"
#include "Drivebase.hpp"
#include "Hardware/HardwareAccess.hpp"

#include <Utils/Vec.hpp>

namespace p28 {

/*
	@struct Driver holds all component drivers
	(motors, leds, speakers, etc..). It provides
	a State interface which is used to Drive the robot.

	@note Here Drive means creating an ActionState
			which can be written to the hardware
*/
struct Driver {
	// Driver state it contains the state of
	// every actuator in human readable / SI units
	struct State {
		mt::Vec2 wheel_vel; // m / s

		State() = default;
	};
	using Command = mt::ValTarg<State>;

	// Indexed with kRightWheel and kLeftWheel
	MotorDriver wheelMotors[2]; 

	// Create the action state that can be writen to 
	// the robot from the current driver state and a target driver state
	Pair<ActionState, Driver> make_actionState(Command command, float delta_s) const;

	static State driver_state(SensorState const& prev, SensorState const& current, float delta_s);
};

} // !p28


#endif