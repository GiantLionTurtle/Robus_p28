
#include "PID.hpp"

#include <Arduino.h>

namespace p28 {

Error update_error(Error err, float value, float setpoint, float delta_s)
{
	Error out;
	out.error = setpoint - value;
	out.diff_error = (out.error - err.error) / delta_s;
	out.sum_error = err.sum_error + (err.error+out.error)/2.0 * delta_s;
	return out;
}
float get(PID const& pid, Error const& error)
{
	// Serial.print("error: ");
	// Serial.println(error.error);
	// Serial.print(",  ");
	// Serial.print(error.sum_error);
	// Serial.print(",  ");
	// Serial.println(error.diff_error);
	return pid.P * error.error + pid.D * error.diff_error + pid.I * error.sum_error;
}

} // !p28