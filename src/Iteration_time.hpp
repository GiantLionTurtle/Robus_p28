
#ifndef P28_ITERATION_TIME_HPP_
#define P28_ITERATION_TIME_HPP_

#include <Arduino.h>

struct Iteration_time {
	float delta_s { 0 }; // Delta time in second since last iteration
	unsigned long time_ms { 0 }; // Time of the system

	static Iteration_time first()
	{
		Iteration_time new_it;
		new_it.time_ms = millis();
		new_it.delta_s = 0.0;
		return new_it;
	}
	Iteration_time current() const
	{
		Iteration_time new_it;
		new_it.time_ms = millis();
		new_it.delta_s = static_cast<float>(new_it.time_ms - time_ms) / 1000.0f;
		return new_it;
	}
};

#endif