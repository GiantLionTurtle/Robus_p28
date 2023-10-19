
#include "Robot.hpp"
#include <Arduino.h>

namespace p28 {

// Adjust the drivebase with additional info such as the position in the field
DrivebaseState adjustDrivebase(DrivebaseState drvbState, SensorState const&  currSensState, 
								GameState const&  prevRobot, GameState const&  robot);

Robot Robot::next(  SensorState prevSensState, SensorState currSensState, 
				    GameState prevGmState, GameState gmState) const
{

	Robot newRobot = *this;
	// Compute delta time
	newRobot.time_ms = millis();
	newRobot.delta_s = static_cast<float>(newRobot.time_ms - time_ms) / 1000.f;

	// New state given the new encoder data
	newRobot.drvb.state 
			= newRobot.drvb.state.update(prevSensState.encoders_ticks, currSensState.encoders_ticks, newRobot.delta_s);
	// Adjust drivebase with other sensors and knowledge of the game
	newRobot.drvb.state = adjustDrivebase(newRobot.drvb.state, currSensState, prevGmState, gmState);


    return newRobot;
}

DrivebaseState adjustDrivebase(DrivebaseState drvbState, SensorState const& currSensState, 
								GameState const& prevRobot, GameState const& robot)
{
	// &&Figureout&& how to adjust the drivebase with auxilary sensors
	return drvbState;
}

} // !p28