
#include "Robot.hpp"
#include <Arduino.h>
#include "Constants.hpp"
#include "Field.hpp"
#include "Sensors/LineDetector.hpp"

#include "Paths.hpp"

namespace p28 {


Robot Robot::initial()
{
	Robot robot;
	robot.drvb.leftWheel.pid = { 1.4, 35.5555, 0.03333333 };
	robot.drvb.rightWheel.pid = { 1.4, 35.5555, 0.03333333 };
	// robot.drvb.concrete.headingPID = { 0.4, 0.18, 0.006 };
	robot.drvb.headingPID = { 0.3, 0.135, 0.0045 };
	robot.drvb.drvMode = Drivebase::followPath;
	robot.drvb.path = Paths::gen_test();
	
	return robot;
}

void Robot::generate_next(  SensorState prevSensState, SensorState currSensState, Iteration_time it_time)
{
	drvb.update(currSensState, prevSensState, it_time);
	going_home = bin.is_full();
}
HardwareState Robot::generate_hardwareState()
{
	HardwareState hrdwState;
	hrdwState = drvb.aggregate(hrdwState);
	hrdwState = bin.Aggregate_hardwareState(hrdwState);
	return hrdwState;
}

void Robot::adjustDrivebase(SensorState const& currSensState,  SensorState const& prevSensState, Iteration_time it_time)
{
	if(Robot::going_home && drvb.finish && drvb.drvMode == Drivebase::Drivemodes::followLine)
	{
		drvb.pos = mt::Vec2(0, 0);
		drvb.heading = mt::Vec2(1, 0);
		char color_bidon;
		switch(color_bidon)
		{
			case kRed:
				drvb.set_path(Paths::generateDropRed(), it_time);
				break;
			case KGreen:
				drvb.set_path(Paths::generateDropGreen(), it_time);
				break;
			case kBlue:
				drvb.set_path(Paths::generateDropBlue(), it_time);
				break;
			case kYellow:
				drvb.set_path(Paths::generateDropYellow(), it_time);
				break;
			default:
				drvb.set_path(Paths::generateDropAll(), it_time);
				break;
		}
	}
}

} // !p28