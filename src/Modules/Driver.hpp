
#ifndef P28_DRIVER_HPP_
#define P28_DRIVER_HPP_

#include "Hardware/MotorDriver.hpp"
#include "Drivebase.hpp"
#include "Hardware/HardwareAccess.hpp"

#include <Utils/Vec.hpp>

namespace p28 {

struct Driver {
	// Description of a drive
	struct State {
		mt::Vec2 wheel_vel; // m / s

		State() = default;
	};
	using Command = mt::ValTarg<State>;

	MotorDriver wheels[2]; // Indexed with kRightWheel and kLeftWheel

	// Create the action state that can be writen to 
	// the robot from the current driver state and a target driver state
	Pair<ActionState, Driver> make_actionState(Command command, float delta_s) const;

	static State driver_state(SensorState const& prev, SensorState const& current, float delta_s);
};

} // !p28


#endif