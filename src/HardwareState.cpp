#include "Utils/Vec.hpp"
#include "HardwareState.hpp"
#include <LibRobus.h>
#include "Constants.hpp"
#include "Robot.hpp"

namespace p28 {

void set_hardwareState (HardwareState hwst)
{
    MOTOR_SetSpeed (RIGHT, hwst.motors.right);      //Sets the motors speed according to the hardware state received
    MOTOR_SetSpeed (LEFT, hwst.motors.left);

    SERVO_SetAngle (kArm_servoId, hwst.armAngle);  //Sets the angle of the servomotor of the arm 
    SERVO_SetAngle (kCup_servoId, hwst.cupAngle);  //Sets the angle of the servomotor controlling the cup "holder"
}

Pair<HardwareState, Robot> generate_hardwareState(ActionState actState, Robot robot)
{
    HardwareState gen_hwst;
    if (actState.openArm) {
        gen_hwst.armAngle = kArm_openAngle;
    }
    if (actState.releaseCup) {
        gen_hwst.cupAngle = kCup_openAngle;
    }
    tie(gen_hwst.motors, robot.drvb) = robot.drvb.hardware_output(actState.path.current(), robot.time_ms, robot.delta_s);
    return { gen_hwst, robot };
}

} // !p28
