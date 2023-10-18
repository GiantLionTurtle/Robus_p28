
#ifndef P28_ROBOTSTATE_HPP_
#define P28_ROBOTSTATE_HPP_

#include "Utils/Vec.hpp"

// Essentialy proprioception for the robot
struct RobotState {
    float delta_s; // Delta time in second since last iteration
};

#endif