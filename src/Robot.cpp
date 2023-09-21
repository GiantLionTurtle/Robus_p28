
#include "Robot.hpp"

namespace p28 {

Robot Robot::snapshot() const
{
	SensorState newSensorState = readSensors();
	Robot_snapshot newSnapshot;
	float delta_s = static_cast<float>(newSensorState.time_ms - time_ms()) / 1000.0f;

	newSnapshot.driverState = Driver::driver_state(sensState, newSensorState, delta_s);

	// The future is now,, current is the prev of the next robot instance
	newSnapshot.location = 
			Drivebase::kinematics(	current.location, 
									newSnapshot.driverState.wheel_vel.left, 
									newSnapshot.driverState.wheel_vel.right,
									delta_s);
	
	// current becomes previous
	return Robot { driver, newSensorState, current, newSnapshot, target, delta_s };
}
time_t Robot::time_ms() const
{
	return sensState.time_ms;
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


void Robot::Ziegler_Nichols_ultimateGain_drivebase(float p_gain, long unsigned int test_duration_ms)
{
	SensorState prev = readSensors();
	SensorState curr;

	float target_vel = 0.5; // m/s
	unsigned int granularity_ms = 10;

	Serial.print("Start P_gain = ");
	Serial.println(p_gain);

	driver.wheelMotors[kLeftMotor].pid.P = p_gain;
	driver.wheelMotors[kRightMotor].pid.P = p_gain;
	unsigned int n_it = (test_duration_ms / granularity_ms);
	for(unsigned int i = 0; i < n_it; ++i) {
		curr = readSensors();

		float diff_s = static_cast<float>(curr.time_ms-prev.time_ms) / 1000.0f;
		Driver::State dState = Driver::driver_state(prev, curr, diff_s);
		ActionState todo;
		tie(todo, driver) = driver.make_actionState({Driver::State{mt::Vec2(target_vel)}, dState}, diff_s);

		// Serial.print("Todo left: ");
		// Serial.println(todo.driveBaseMotor.left);

		prev = curr;
		writeActions(todo);
		Serial.print(curr.time_ms);
		Serial.print(", ");
		Serial.print(dState.wheel_vel.left);
		Serial.print(", ");
		Serial.println(dState.wheel_vel.right);
		delay(granularity_ms);
	}
	writeActions(ActionState());
	delay(20);
	Serial.println("End all tests");
}

} // !p28