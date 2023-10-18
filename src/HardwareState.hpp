#ifndef P28_HardwareState_
#define P28_HardwareState_

#include "Utils/Vec.hpp"
#include "ActionState.hpp"
const int ANGLEARM = 15;
const int ANGLECUP = 15;

struct HardwareState {
p28::mt::Vec2 motors; // Values from [-1,1]
int angleArm {0};   //angle of the arm 
int angleCup {0};   //angle of the servomotor that holds the cup
};

void set_hardwareState(struct HardwareState hwst);

HardwareState generate_hardwareState(ActionState actState);

#endif