
#include "Robot.hpp"

namespace p28 {

Robot Robot::snapshot() const
{
	SensorState newSensorState = readSensors();
	Robot_snapshot newSnapshot;
	time_t new_time = millis();
	float delta_s = static_cast<float>(new_time - time_ms) / 1000.0f;

	newSnapshot.driverState = Driver::driver_state(sensState, newSensorState, delta_s);

	// The future is now,, current is the prev of the next robot instance
	newSnapshot.location = 
			Drivebase::kinematics(	current.location, 
									newSnapshot.driverState.wheel_vel.left, 
									newSnapshot.driverState.wheel_vel.right,
									delta_s);
	
	// current becomes previous
	return Robot { driver, newSensorState, current, newSnapshot, target, new_time, delta_s };
}
Pair<ActionState, Robot> Robot::next_action() const
{
	ActionState toDo;
	Driver newDriver;
	tie(toDo, newDriver) = driver.make_actionState(drive_command(), deltaTime_s);

	return { 	toDo, 
				Robot { newDriver, sensState, previous, current, target } };	
}

Driver::Command Robot::drive_command() const
{
	mt::Vec2 targetVel = Drivebase::inverse_kinematics(current.location, target.location.pos, kMaxVel);
	return {
		Driver::State { current.driverState.wheel_vel }, // Current
		Driver::State { targetVel }  // Target
	};
}

} // !p28