
#ifndef P28_TESTPID_HPP_
#define P28_TESTPID_HPP_

#include "PID.hpp"

namespace p28 {

namespace TestPID {

void Ziegler_Nichols(float P_min, float target_speed);

void test_pid_straightLine(PID pid_left, PID pid_right, float target_velocity);

} // !TestPID

} // !p28

#endif