#include "Utils/Vec.hpp"
#include "HardwareState.hpp"
#include <LibRobus.h>
#include "Constants.hpp"


void set_hardwareState (HardwareState hwst)
{
    MOTOR_SetSpeed (RIGHT, hwst.motors.right);      //Sets the motors speed according to the hardware state received
    MOTOR_SetSpeed (LEFT, hwst.motors.left);

    SERVO_SetAngle (kArm_servoId, hwst.angleArm);  //Sets the angle of the servomotor of the arm 
    SERVO_SetAngle (kCup_servoId, hwst.angleCup);  //Sets the angle of the servomotor controlling the cup "holder"
}

HardwareState generate_hardwareState(ActionState actState)
{
    
}
