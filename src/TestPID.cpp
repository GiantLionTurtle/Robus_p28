
#include "TestPID.hpp"

#include <Arduino.h>

#include <LibRobus.h>
#include "PID.hpp"
#include "Drivebase.hpp"

#define ITERATION_STEPS 400

namespace p28 {

namespace TestPID {


struct DataPoint {
	unsigned int timestamp;
	float out_left;
	float speed_left;
	// float out_right;
};

void Ziegler_Nichols(float P, float target_velocity)
{
	DataPoint testData[ITERATION_STEPS];
	testData[0].timestamp = millis();
	testData[0].out_left = 0;
	testData[0].speed_left = 0;

	int32_t ticks_left = ENCODER_Read(LEFT);
	int32_t ticks_left_prev = ENCODER_Read(LEFT);
	Error error_left;

	int32_t ticks_right = ENCODER_Read(RIGHT);
	int32_t ticks_right_prev = ENCODER_Read(RIGHT);
	Error error_right;
	
	PID motorPID { P, 0.0, 0.0 };

	for(int i = 1; i < ITERATION_STEPS; ++i) {
		delay(1);
		testData[i].timestamp = millis();

		float delta_s = static_cast<float>(testData[i].timestamp - testData[i-1].timestamp) / 1000.0f;

		ticks_left = ENCODER_Read(LEFT);
		float velocity_left = ticks_to_dist(ticks_left-ticks_left_prev) / delta_s;
		error_left = update_error(error_left, velocity_left, target_velocity, delta_s);
		float out_left = get(motorPID, error_left);

		ticks_right = ENCODER_Read(RIGHT);
		float velocity_right = ticks_to_dist(ticks_right-ticks_right_prev) / delta_s;
		error_right = update_error(error_right, velocity_right, target_velocity, delta_s);
		float out_right = get(motorPID, error_right);

		ticks_left_prev = ticks_left;
		ticks_right_prev = ticks_right;

		MOTOR_SetSpeed(LEFT, out_left);
		MOTOR_SetSpeed(RIGHT, out_right);
		testData[i].out_left = out_left;// * 100;
		testData[i].speed_left = velocity_left;// * 100;
	}

	MOTOR_SetSpeed(LEFT, 0.0);
	MOTOR_SetSpeed(RIGHT, 0.0);

	Serial.print(" -------- WITH P = ");
	Serial.print(P);
	Serial.println(" -------- ");
	for(int i = 0; i < ITERATION_STEPS; ++i) {
		Serial.print(testData[i].timestamp);
		Serial.print(",  ");
		Serial.print(testData[i].out_left, 8);
		Serial.print(",  ");
		Serial.println(testData[i].speed_left, 8);
	}
}

void test_pid_straightLine(PID pid_left, PID pid_right, float target_velocity)
{
	// Try going 10m and output ticks dist difference

	int32_t ticks_left = ENCODER_Read(LEFT);
	int32_t ticks_init_left = ENCODER_Read(LEFT);
	int32_t ticks_left_prev = ENCODER_Read(LEFT);
	Error error_left;

	int32_t ticks_right = ENCODER_Read(RIGHT);
	int32_t ticks_right_prev = ENCODER_Read(RIGHT);
	Error error_right;

	unsigned long timestamp_prev = millis();

	while(ticks_to_dist(abs(ticks_init_left-ticks_left)) < 1.0) {
		delay(1);
		unsigned long timestamp = millis();

		float delta_s = static_cast<float>(timestamp - timestamp_prev) / 1000.0f;

		ticks_left = ENCODER_Read(LEFT);
		float velocity_left = ticks_to_dist(ticks_left-ticks_left_prev) / delta_s;
		error_left = update_error(error_left, velocity_left, target_velocity, delta_s);
		float out_left = get(pid_left, error_left);

		ticks_right = ENCODER_Read(RIGHT);
		float velocity_right = ticks_to_dist(ticks_right-ticks_right_prev) / delta_s;
		error_right = update_error(error_right, velocity_right, target_velocity, delta_s);
		float out_right = get(pid_right, error_right);

		ticks_left_prev = ticks_left;
		ticks_right_prev = ticks_right;

		MOTOR_SetSpeed(LEFT, out_left);
		MOTOR_SetSpeed(RIGHT, out_right);
	}

	Serial.println(" ---------- Test completed ---------- ");
	Serial.print("Left ticks: ");
	Serial.println(ticks_left);
	Serial.print("Right ticks: ");
	Serial.println(ticks_right);
	Serial.print("Diff: ");
	Serial.println(ticks_left-ticks_right);

	MOTOR_SetSpeed(LEFT, 0.0);
	MOTOR_SetSpeed(RIGHT, 0.0);
}

} // !TestPID

} // !p28