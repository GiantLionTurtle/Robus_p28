
#ifndef P28_PID_HPP_
#define P28_PID_HPP_



struct PID {
	double P;
	double I;
	double D;
};
struct Error {
	double error { 0.0 };
	double diff_error { 0.0 };
	double sum_error { 0.0 };
};

struct Error update_error(struct Error err, double value, double setpoint, double delta_s);
double get(struct PID const& pid, struct Error const& error);



#endif