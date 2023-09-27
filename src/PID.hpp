
#ifndef P28_PID_HPP_
#define P28_PID_HPP_

namespace p28 {

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

Error update_error(Error err, float value, float setpoint, float delta_s);
float get(PID const& pid, Error const& error);

} // !p28

#endif