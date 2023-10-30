#include "Utils/Vec2.hpp"
#include "HardwareState.hpp"
#include <LibRobus.h>
#include "Constants.hpp"
#include "Robot.hpp"

namespace p28 {

HardwareState HardwareState::mix(HardwareState hrdwState) const
{
	hrdwState.motors = hrdwState.motors * kMotorHarwareStateMixFactor + motors * (1-kMotorHarwareStateMixFactor);
	return hrdwState;
}

void set_hardwareState (HardwareState hwst)
{
	MOTOR_SetSpeed (RIGHT, hwst.motors.right);      //Sets the motors speed according to the hardware state received
	MOTOR_SetSpeed (LEFT, hwst.motors.left);

	SERVO_SetAngle (kArm_servoId, hwst.armAngle);  //Sets the angle of the servomotor of the arm 
	SERVO_SetAngle (kCup_servoId, hwst.cupAngle);  //Sets the angle of the servomotor controlling the cup "holder"
}

void printHarwareState(HardwareState state)
{
    Serial.print("motors:");
    print(state.motors);
    Serial.print(" | Arm angle:");
    Serial.print(state.armAngle);
    Serial.print(" | Cup angle:");
    Serial.println(state.cupAngle);

}

HardwareState generate_hardwareState(Robot robot)
{
	HardwareState gen_hwst;
	if (robot.openArm) {
		gen_hwst.armAngle = kArm_openAngle;
	}
	if (robot.releaseCup) {
		gen_hwst.cupAngle = kCup_openAngle;
	}
	gen_hwst.motors = robot.drvb.concrete.hardware_output();
	return gen_hwst;
}

} // !p28
