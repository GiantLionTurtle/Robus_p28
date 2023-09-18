
#include "HardwareTest.hpp"

#include "HardwareAccess.hpp"

#include <Arduino.h>

namespace p28 {

ActionState greenLightsOn()
{
	// figure out how to turn green leds on!
	return ActionState();
}
ActionState redLightsOn()
{
	// figure out how to turn red leds on!
	return ActionState();
}

void flash_lights(ActionState on)
{
	for(int i = 0; i < 4; ++i) {
		writeActions(on); // on
		
		delay(50);

		writeActions(ActionState()); // off

		delay(50);
	}	
}

void test_lights()
{
	flash_lights(greenLightsOn());
	delay(1000);
	flash_lights(redLightsOn());
}

bool test_turn_right()
{
	SensorState current = readSensors();
	ActionState actState;
	actState.driveBaseMotor = { 0.5, -0.5 };

	writeActions(actState);

	delay(1000);

	writeActions(ActionState()); // Stop everything

	SensorState sensorDiff = readSensors() - current;
	if(sensorDiff.driveEncoders.left > sensorDiff.driveEncoders.right)
		return true; // Success
	
	return false;
}
bool test_turn_left()
{
	SensorState current = readSensors();
	ActionState actState;
	actState.driveBaseMotor = { 0.5, -0.5 };

	writeActions(actState);

	delay(1000);

	writeActions(ActionState()); // Stop everything

	SensorState sensorDiff = readSensors() - current;
	if(sensorDiff.driveEncoders.left < sensorDiff.driveEncoders.right)
		return true; // Success
	
	return false;
}
bool test_forward()
{
	SensorState current = readSensors();
	ActionState actState;
	actState.driveBaseMotor = { 0.5, 0.5 };

	writeActions(actState);

	delay(5000);

	writeActions(ActionState()); // Stop everything

	SensorState sensorDiff = readSensors() - current;
	if(sensorDiff.driveEncoders.left > 0 && sensorDiff.driveEncoders.right > 0)
		return true; // Success
	
	return false;
}
bool test_backward()
{
	SensorState current = readSensors();
	ActionState actState;
	actState.driveBaseMotor = { -0.5, -0.5 };

	writeActions(actState);

	delay(5000);

	writeActions(ActionState()); // Stop everything

	SensorState sensorDiff = readSensors() - current;
	if(sensorDiff.driveEncoders.left < 0 && sensorDiff.driveEncoders.right < 0)
		return true; // Success
	
	return false;
}
void indicate_pass(bool pass)
{
	if(pass) {
		flash_lights(greenLightsOn());
	} else {
		flash_lights(redLightsOn());
	}
}
void test_motors_and_encoders()
{
	indicate_pass(test_turn_right());
	delay(1000);

	indicate_pass(test_turn_left());
	delay(1000);

	indicate_pass(test_forward());
	delay(1000);

	indicate_pass(test_backward());
	delay(1000);

}

void test_all()
{
	test_lights();

	delay(1000);

	test_motors_and_encoders();
}

} // !p28