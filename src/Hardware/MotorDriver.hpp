
#ifndef P28_MOTORDRIVER_HPP_
#define P28_MOTORDRIVER_HPP_

#include <Utils/Pair.hpp>
#include <Utils/Vec.hpp>

namespace p28 {


struct Error {
	float error { 0.0 };
	float error_change { 0.0 };
	float error_sum { 0.0 };
};
struct PID {
	float P { 0.0 };
	float I { 0.0 };
	float D { 0.0 };
	float F { 0.0 };

	float get(Error error) const;
};


struct MotorDriver {
	Error speederror;
	PID pid;


	// Compute the value [-1, 1] to 
	// send the board based on
	Pair<float, MotorDriver> drive(mt::ValTarg<float> const& valTarg, float time_s) const;
};

}

#endif