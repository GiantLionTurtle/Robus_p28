
#include <Arduino.h>
#include "LibRobus.h"
#include "Constants.hpp"

struct PID {
	float P;
	float I;
	float D;
};
struct Error {
	float error { 0.0 };
	float diff_error { 0.0 };
	float sum_error { 0.0 };
};

Error update_error(Error err, float value, float setpoint, float delta_s)
{
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

unsigned long prev_time_ms;
float set_speed = 0.4; // m/s
Error left_velocity_error;
Error right_velocity_error;

PID motorPID;
uint32_t left_prev_ticks = 0;
uint32_t right_prev_ticks = 0;


float ticks_to_dist(uint32_t ticks)
{
	return static_cast<float>(ticks) / 3200.0 * TWO_PI * p28::kWheelRadius;
}

void setup() 
{
	BoardInit();
	motorPID.P = 1.4;
	motorPID.I = 35.5555555555556;
	motorPID.D = 0.03333333333333;
	motorPID.P = 4;
	Serial.begin(9600);
	delay(1000);
	Serial.println("Begin!\n");
	prev_time_ms = millis();
	delay(10);
}

void loop() 
{
	unsigned long time_ms = millis();

	float delta_s = static_cast<float>(time_ms - prev_time_ms) / 1000.0f;

	uint32_t left_current_ticks = ENCODER_Read(LEFT);
	float left_current_speed = ticks_to_dist(left_current_ticks-left_prev_ticks) / delta_s;
	left_velocity_error = update_error(left_velocity_error, left_current_speed, set_speed, delta_s);

	float left_motorset = get(motorPID, left_velocity_error);

	uint32_t right_current_ticks = ENCODER_Read(RIGHT);
	float right_current_speed = ticks_to_dist(right_current_ticks-right_prev_ticks) / delta_s;
	right_velocity_error = update_error(right_velocity_error, right_current_speed, set_speed, delta_s);

	float right_motorset = get(motorPID, right_velocity_error);


	MOTOR_SetSpeed(LEFT, left_motorset);
	MOTOR_SetSpeed(RIGHT, right_motorset);

	left_prev_ticks = left_current_ticks;
	right_prev_ticks = right_current_ticks;
	prev_time_ms = time_ms;

	// Serial.print(time_ms);
	// Serial.print(", ");
	// Serial.print(motorset);
	// Serial.print(", ");
	// Serial.println(left_velocity_error.error);
	// Serial.print(", ");
	// Serial.println(current_ticks);

	delay(10);
}