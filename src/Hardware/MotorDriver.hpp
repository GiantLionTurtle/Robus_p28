
#ifndef P28_MOTORDRIVER_HPP_
#define P28_MOTORDRIVER_HPP_

#include <Utils/Pair.hpp>
#include <Utils/Vec.hpp>

namespace p28 {

/*
	@struct Error is a compound error
	to be used in a PID controller
*/
struct Error {
	float error { 0.0 };
	float error_change { 0.0 };
	float error_sum { 0.0 };
};

/*
	@struct PID defines a 
	Proportional
	Integral
	Derivative

	Controller to reach a target
*/
struct PID {
	float P { 0.0 };
	float I { 0.0 };
	float D { 0.0 };
	float F { 0.0 };

	// Returns the output of the pid
	// controller for a given compound error
	float get(Error error) const;
};

/*
	@struct MotorDriver holds a velocity error
	and a pid controller. It is intended to 
	bridge the target velocity in m/s of each 
	wheel and translate it in a range [-1, 1]
	which is inteligible to the LibRobUS
*/
struct MotorDriver {
	Error velocityError;
	PID pid;

	// Compute the value [-1, 1] to 
	// send the board based on
	Pair<float, MotorDriver> drive(mt::ValTarg<float> const& valTarg, float time_s) const;
};

}

#endif