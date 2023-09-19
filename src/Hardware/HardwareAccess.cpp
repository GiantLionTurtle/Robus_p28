
#include "HardwareAccess.hpp"

#include <Constants.hpp>
#include <LibRobus.h>

namespace p28 {

void init()
{
    BoardInit();
}
SensorState readSensors()
{
	SensorState out;
	out.driveEncoders.left 	= ENCODER_Read(kLeftMotor);
	out.driveEncoders.right	= ENCODER_Read(kRightMotor);

	out.bumperSwitches[kLeftBumper] 	= ROBUS_IsBumper(kLeftBumper);
	out.bumperSwitches[kRightBumper] 	= ROBUS_IsBumper(kRightBumper);
	out.bumperSwitches[kFrontBumper] 	= ROBUS_IsBumper(kFrontBumper);
	out.bumperSwitches[kRearBumper] 	= ROBUS_IsBumper(kRearBumper);
	return out;
}
void SensorState::print() const
{
	Serial.print("Sensor state [");
	Serial.print(time_ms);
	Serial.println("]");

	Serial.print("Encoders\t\t[");
	Serial.print(driveEncoders.left);
	Serial.print(",  ");
	Serial.print(driveEncoders.right);
	Serial.println("]");

	Serial.print("Bumpers\t\t[");
	Serial.print(bumperSwitches[0]);
	Serial.print(",  ");
	Serial.print(bumperSwitches[1]);
	Serial.print(",  ");
	Serial.print(bumperSwitches[2]);
	Serial.print(",  ");
	Serial.print(bumperSwitches[3]);
	Serial.println("]");
}

void writeActions(ActionState const& act)
{
	// Library calls to write the action state
	MOTOR_SetSpeed(kLeftMotor, act.driveBaseMotor.left);
	MOTOR_SetSpeed(kLeftMotor, act.driveBaseMotor.right);
}

SensorState operator-(SensorState const& lhs, SensorState rhs)
{
	SensorState out;
	out.driveEncoders = lhs.driveEncoders - rhs.driveEncoders;

	for(int i = 0; i < 4; ++i) {
		out.bumperSwitches[i] = lhs.bumperSwitches[i] - rhs.bumperSwitches[i];
	}
}

}