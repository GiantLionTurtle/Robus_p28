
#include "Robot.hpp"
#include <Arduino.h>
#include "Constants.hpp"
#include "Field.hpp"
#include "LineDetector.hpp"

#include "Paths.hpp"

namespace p28 {


Robot Robot::initial(GameState gmState)
{
	Robot robot;
	robot.drvb.concrete.left.pid = { 1.4, 35.5555, 0.03333333 };
	robot.drvb.concrete.right.pid = { 1.4, 35.5555, 0.03333333 };
	// robot.drvb.concrete.headingPID = { 0.4, 0.18, 0.006 };
	robot.drvb.concrete.headingPID = { 0.3, 0.135, 0.0045 };
	robot.drvb.followLine = true;

	
	return robot;
}

void Robot::generate_next(  SensorState prevSensState, SensorState currSensState, 
				   			 GameState prevGmState, GameState gmState, Iteration_time it_time)
{
	drvb.update(currSensState, prevSensState, it_time);
}



void Robot::adjustDrivebase(SensorState const& currSensState,  SensorState const& prevSensState,
								GameState const& prevGmState, GameState const& gmState)
{
}
	










} // !p28