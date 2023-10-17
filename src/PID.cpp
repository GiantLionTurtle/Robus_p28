
#include "PID.hpp"

#include <Arduino.h>



struct Error update_error(struct Error err, double value, double setpoint, double delta_s)
{
	// // Serial.print("Delta s: ");
	// // Serial.println(delta_s);
	Error out;
	out.error = setpoint - value;
	out.diff_error = (out.error - err.error);// / delta_s;
	out.sum_error = err.sum_error + out.error * delta_s;
	return out;
}
double get(struct PID const& pid, struct Error const& error)
{
	return pid.P * error.error + pid.D * error.diff_error + pid.I * error.sum_error;
}



