#include "Utils/Vec2.hpp"
#include "HardwareState.hpp"
#include <LibRobus.h>
#include "Constants.hpp"
#include "Robot.hpp"

#define CUP_SERVO_ID 0
#define ARM_SERVO_ID 1

namespace p28 {

HardwareState HardwareState::mix(HardwareState hrdwState) const
{
	hrdwState.motors = hrdwState.motors * kMotorHarwareStateMixFactor + motors * (1-kMotorHarwareStateMixFactor);
	if(abs(hrdwState.motors.left) <= 0.05 && abs(hrdwState.motors.right) <= 0.05)
		hrdwState.motors = { 0, 0 };
	return hrdwState;
}
HardwareState HardwareState::initial()
{
	HardwareState out;
	out.motors = { 0.0f, 0.0f };
	out.armAngle = kArm_closeAngle;
	out.cupAngle = kCup_closeAngle;
	return out;
}

void set_hardwareState (HardwareState hwst)
{
	MOTOR_SetSpeed (RIGHT, hwst.motors.right);      //Sets the motors speed according to the hardware state received
	MOTOR_SetSpeed (LEFT, hwst.motors.left);

	if(hwst.armAngle != -1)
		SERVO_SetAngle (ARM_SERVO_ID, hwst.armAngle);  //Sets the angle of the servomotor of the arm 
	if(hwst.cupAngle != -1)
		SERVO_SetAngle (CUP_SERVO_ID, hwst.cupAngle);  //Sets the angle of the servomotor controlling the cup "holder"
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

HardwareState generate_hardwareState(Robot const& robot)
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
