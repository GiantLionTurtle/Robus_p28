
#include "TestPID.hpp"

#include <Arduino.h>

#include <LibRobus.h>
#include "PID.hpp"
#include "Drivebase.hpp"



void init(double P, double speed_m_s)
{
    unsigned long prev_time_ms;
    double set_speed = 0.4; // m/s
    Error left_velocity_error;
    Error right_velocity_error;

    PID motorPID;
    uint32_t left_prev_ticks = 0;
    uint32_t right_prev_ticks = 0;


    BoardInit();
	motorPID.P = 1.4;
	motorPID.I = 35.5555555555556;
	motorPID.D = 0.03333333333333;
	motorPID.P = 4;
	// Serial.begin(9600);
	delay(1000);
	// Serial.println("Begin!\n");
	prev_time_ms = millis();
	delay(10);


    unsigned long time_ms = millis();

	double delta_s = static_cast<float>(time_ms - prev_time_ms) / 1000.0f;

	uint32_t left_current_ticks = ENCODER_Read(LEFT);
	double left_current_speed = ticks_to_dist(left_current_ticks-left_prev_ticks) / delta_s;
	left_velocity_error = update_error(left_velocity_error, left_current_speed, set_speed, delta_s);

	double left_motorset = get(motorPID, left_velocity_error);

	uint32_t right_current_ticks = ENCODER_Read(RIGHT);
	double right_current_speed = ticks_to_dist(right_current_ticks-right_prev_ticks) / delta_s;
	right_velocity_error = update_error(right_velocity_error, right_current_speed, set_speed, delta_s);

	double right_motorset = get(motorPID, right_velocity_error);


	MOTOR_SetSpeed(LEFT, left_motorset);
	MOTOR_SetSpeed(RIGHT, right_motorset);

	left_prev_ticks = left_current_ticks;
	right_prev_ticks = right_current_ticks;
	prev_time_ms = time_ms;

	delay(10);
}

