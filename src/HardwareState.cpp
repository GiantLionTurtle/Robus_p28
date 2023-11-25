#include "Utils/Vec2.hpp"
#include "HardwareState.hpp"
#include <LibRobus.h>
#include "Constants.hpp"
#include "Robot.hpp"
#include "ExtraMotors.hpp"


#define CUP_SERVO_ID 0
#define ARM_SERVO_ID 1

namespace p28 {

static ExtraMotors extra_motors;

HardwareState HardwareState::mix(HardwareState hrdwState) const
{
	hrdwState.motors = hrdwState.motors * kMotorHarwareStateMixFactor + motors * (1-kMotorHarwareStateMixFactor);
	if(abs(hrdwState.motors.left) <= 0.05 && abs(hrdwState.motors.right) <= 0.05)
		hrdwState.motors = { 0, 0 };
	return hrdwState;
}
HardwareState HardwareState::initial()
{
	extra_motors.init();
	HardwareState out;
	out.motors = { 0.0f, 0.0f };
	
	return out;
}

void set_hardwareState (HardwareState hwst)
{
	if(abs(hwst.motors.left) == 1.0 || abs(hwst.motors.right) == 1.0) {
		MOTOR_SetSpeed (RIGHT, 0.0);
		MOTOR_SetSpeed (LEFT, 0.0);
		Serial.println("motor speeds out of bounds");

	} else {
		// Serial.print("Set ");
		// mt::println(hwst.motors);
		MOTOR_SetSpeed (RIGHT, hwst.motors.right);      //Sets the motors speed according to the hardware state received
		MOTOR_SetSpeed (LEFT, hwst.motors.left);
	}

	SERVO_SetAngle (0 , hwst.clawAngle);
	SERVO_SetAngle (1, hwst.armAngle);

	int n_steps = mt::clamp(hwst.conveyorSteps-extra_motors.conveyor_steps, -10, 10);
	extra_motors.conveyor.step(n_steps);
	extra_motors.conveyor_steps += n_steps;

	extra_motors.trap.write(hwst.trapAngle);
	extra_motors.bin_red.write(hwst.bin_select_angles [kRed]);
	extra_motors.bin_green.write(hwst.bin_select_angles [KGreen]);
	extra_motors.bin_blue.write(hwst.bin_select_angles [kBlue]);
 
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
