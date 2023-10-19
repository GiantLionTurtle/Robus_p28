#ifndef P28_HARDWARESTATE_HPP_
#define P28_HARDWARESTATE_HPP_

#include "Utils/Vec.hpp"
#include "ActionState.hpp"

namespace p28 {

struct HardwareState {
	p28::mt::Vec2 motors; // Values from [-1,1]
	int armAngle { kArm_closeAngle };   //angle of the arm 
	int cupAngle { kCup_closeAngle };   //angle of the servomotor that holds the cup
};

void set_hardwareState(struct HardwareState hwst);

Pair<HardwareState, Robot> generate_hardwareState(ActionState actState, Robot robot);

} // !p28

#endif