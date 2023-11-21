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

	return out;
}

void set_hardwareState (HardwareState hwst)
{
	MOTOR_SetSpeed (RIGHT, hwst.motors.right);      //Sets the motors speed according to the hardware state received
	MOTOR_SetSpeed (LEFT, hwst.motors.left);
	SERVO_SetAngle (0 , hwst.clawAngle);
	SERVO_SetAngle (1, hwst.armAngle);
	hwst.conveyor.setSpeed(5);
	hwst.conveyor.step(hwst.conveyorSteps);
	//SERVO_SetAngle (2, kopen_trap_angle)       implementation of the servo library needed for more servos 
 
}

void print(HardwareState state)
{
    Serial.print("motors:");
    print(state.motors);
    Serial.print(" | Arm angle:");
    Serial.print(state.armAngle);
    Serial.print(" | Cup angle:");
    Serial.println();

}

} // !p28
