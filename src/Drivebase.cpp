
#include "Drivebase.hpp"

#include <LibRobus.h>

namespace p28 {

float ticks_to_dist(uint32_t ticks)
{
	return static_cast<float>(ticks) / 3200.0 * TWO_PI * p28::kWheelRadius;
}

Motor get_motor_speed(Motor motor, float delta_s)
{
	int32_t current_ticks = ENCODER_Read(motor.ID);
	int32_t ticks_diff = current_ticks - motor.last_ticks;

	motor.speed = ticks_to_dist(ticks_diff) / delta_s;
	motor.last_ticks = current_ticks;
	return motor;
}
Motor update_motor_at_speed(Motor motor, float set_speed, long int time_ms)
{
	long int diff_time_ms = time_ms - motor.last_time_ms;
	float delta_s = static_cast<float>(diff_time_ms) / 1000.0f;

	motor.last_time_ms = time_ms;
	motor = get_motor_speed(motor, delta_s);
	motor.error = update_error(motor.error, motor.speed, set_speed, delta_s);
	float harware_set = get(motor.pid, motor.error);
	MOTOR_SetSpeed(motor.ID, harware_set);
	return motor;
}

} // !p28