
#include "MotorDriver.hpp"

namespace p28 {

float PID::get(Error error) const
{
	return error.error * P + error.error_change * D + error.error_sum * I + F;
}

Pair<float, MotorDriver> MotorDriver::drive(mt::ValTarg<float> const& valTarg, float time_s) const
{
	Error newError;
	newError.error = valTarg.target - valTarg.value;
	newError.error_change = velocityError.error - newError.error;
	newError.error_sum = velocityError.error_sum + newError.error * time_s;

	// Serial.print("Error: ");
	// Serial.println(newError.error);

	return { 	pid.get(velocityError), 
				MotorDriver{newError, pid} };
}


}