
#ifndef P28_MISSION_HPP_
#define P28_MISSION_HPP_

#include "Robot.hpp"

/*
    Utilities functions for missions
*/
namespace p28 {
namespace Mission {

struct Epsilon {
    float location2 { 0.01 };
    float front2 { 0.001 };
};  

bool close_enough(Robot_snapshot const& snapshot_, Robot_target const& target, Epsilon epsilon);

} // !Mission
} // p28

#endif