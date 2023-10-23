
#include "Robot.hpp"
#include <Arduino.h>

namespace p28 {

// Adjust the drivebase with additional info such as the position in the field
DrivebaseState adjustDrivebase(DrivebaseState drvbState, SensorState const&  currSensState, 
								GameState const&  prevGmState, GameState const&  gmState);

void ballSwerve_helper(Robot& robot, Objective obj_state);

Robot Robot::generate_next(  SensorState prevSensState, SensorState currSensState, 
				   			 GameState prevGmState, GameState gmState, Iteration_time it_time) const
{
	Robot newRobot;
	// New state given the new encoder data

	newRobot.drvb.state = newRobot.drvb.state.update_kinematics(prevSensState.encoders_ticks, currSensState.encoders_ticks, it_time.delta_s);

	// Adjust drivebase with other sensors and knowledge of the game
	newRobot.drvb.state = adjustDrivebase(newRobot.drvb.state, currSensState, prevGmState, gmState);
	newRobot.drvb.update_path();

	newRobot.drvb.concrete = newRobot.drvb.update_concrete(it_time);

	// Cup zone?
	if(gmState.missionState.knock_cup == Objective::UnderWay) {
		newRobot.openArm = true;
	}

	if(gmState.missionState.trap_ball == Objective::Start) {
		ballSwerve_helper(newRobot, gmState.missionState.trap_ball);
	}

    return newRobot;
}

DrivebaseState adjustDrivebase(DrivebaseState drvbState, SensorState const& currSensState, 
								GameState const& prevGmState, GameState const& gmState)
{
	// &&Figureout&& how to adjust the drivebase with auxilary sensors
	return drvbState;
}

DrivebasePath gen_ballSwervePath(Robot const& robot)
{
	// &&Figureout&&
}
void ballSwerve_helper(Robot& robot, Objective obj_state)
{
	if(obj_state == Objective::Start) {
		robot.drvb.path = gen_ballSwervePath(robot);
	} else if(obj_state == Objective::UnderWay && robot.drvb.path.index == kCupRelease_pathIndex) {
		robot.releaseCup = true;
	}
}


} // !p28