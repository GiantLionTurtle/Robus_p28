
#include "Driver.hpp"
#include "Drivebase.hpp"

#include <Constants.hpp>

namespace p28 {

Pair<ActionState, Driver> Driver::make_actionState(Command command, float delta_s) const
{
	ActionState toDo;
	Driver newDriver;

	tie(toDo.driveBaseMotor[kLeftMotor], newDriver.wheels[kLeftMotor])
		= wheels[kLeftMotor].drive({ command.value.wheel_vel.left, command.target.wheel_vel.left }, delta_s);
	tie(toDo.driveBaseMotor[kRightMotor], newDriver.wheels[kRightMotor])
	 	= wheels[kRightMotor].drive({ command.value.wheel_vel.right, command.target.wheel_vel.right }, delta_s);
	
	// Other subsystems add their driving code here

	return { toDo, newDriver };
}
Driver::State Driver::driver_state(SensorState const& prev, SensorState const& current, float delta_s)
{
	Driver::State out;
	
	out.wheel_vel[kLeftMotor] 	= Drivebase::ticks_to_dist(current.driveEncoders.left) / delta_s;
	out.wheel_vel[kRightMotor] 	= Drivebase::ticks_to_dist(current.driveEncoders.right) / delta_s;
	
	return out;
}

} // !p28