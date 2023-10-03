
#include "PID.hpp"

#include <Arduino.h>

namespace p28 {

Error update_error(Error err, float value, float setpoint, float delta_s)
{
	// Serial.print("Delta s: ");
	// Serial.println(delta_s);
	Error out;
	out.error = setpoint - value;
	out.diff_error = out.error - err.error;
	out.sum_error = err.sum_error * 0.999 + out.error * delta_s;
	return out;
}
float get(PID const& pid, Error const& error)
{
	return pid.P * error.error + pid.D * error.diff_error + pid.I * error.sum_error;
}

} // !p28