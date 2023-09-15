
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
void writeActions(ActionState const& act)
{
	MOTOR_SetSpeed(kLeftMotor, act.driveBaseMotor[kLeftMotor]);
	MOTOR_SetSpeed(kLeftMotor, act.driveBaseMotor[kLeftMotor]);
}

}