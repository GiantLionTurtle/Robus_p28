
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
	newError.error_change = speederror.error - newError.error;
	newError.error_sum += newError.error * time_s;

	return { 	pid.get(speederror), 
				MotorDriver{newError, pid} };
}


}