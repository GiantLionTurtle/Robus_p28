#include "Utils/Vec.hpp"
#include "HardwareState.hpp"
#include <LibRobus.h>
#include "Constants.hpp"

namespace p28 {

void set_hardwareState (HardwareState hwst)
{
    MOTOR_SetSpeed (RIGHT, hwst.motors.right);      //Sets the motors speed according to the hardware state received
    MOTOR_SetSpeed (LEFT, hwst.motors.left);

    SERVO_SetAngle (kArm_servoId, hwst.armAngle);  //Sets the angle of the servomotor of the arm 
    SERVO_SetAngle (kCup_servoId, hwst.cupAngle);  //Sets the angle of the servomotor controlling the cup "holder"
}

HardwareState generate_hardwareState(ActionState actState)
{
    HardwareState gen_hwst;
    if (actState.openArm) {
        gen_hwst.armAngle = kArm_openAngle;
    }
    if (actState.releaseCup) {
        gen_hwst.cupAngle = kCup_openAngle;
    }
}

} // !p28
