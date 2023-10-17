
#ifndef P28_ROBOTSTATE_HPP_
#define P28_ROBOTSTATE_HPP_

#include "Utils/Vec.hpp"

// Essentialy proprioception for the robot
struct RobotState {
    p28::mt::Vec2 pos; // Position in m
    p28::mt::Vec2 heading; // Heading with a length of velocity (m/s)

    p28::mt::Vec2 wheelsVelocities; // Velocity in m/s of each wheel

    float delta_s; // Delta time in second since last iteration
};

#endif