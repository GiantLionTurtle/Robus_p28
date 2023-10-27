
#include "Robot.hpp"
#include <Arduino.h>

#include "Field.hpp"

namespace p28 {

// Adjust the drivebase with additional info such as the position in the field
DrivebaseState adjustDrivebase(DrivebaseState drvbState, SensorState const&  currSensState, 
								GameState const&  prevGmState, GameState const&  gmState);

void ballSwerve_helper(Robot& robot, Objective obj_state);
mt::Vec2 heading_from_ir(mt::Vec2 baseVec, SensorState const& sensState);


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

	/*
		Ways to know where we are

		1. Color change
			a. Guess which color change line we are crossing
			b. Intersect the ray from the robot position, in the robot heading with said line

		2. Line follower
			a. Guess which line we are following
			b. Same as with color change, intersect with line
		
		3. Bumpers in shortcut
			a. If we are in shortcut mission 
			b. If both bumpers are on for the first time
			c. set position
		
		4. IR sensors
			a. If in right zones and in right orientation
			b. Check the wall to compute heading
			c. Set the heading

		
	*/

#ifdef ENABLE_ZONESWITCH_DRIVEBASE_ADJUSTMENTS
	// Zone change
	if(prevGmState.zone == 1 && gmState.zone == 2) {
		return drvbState.intersect_line(Field::zone_1_to_2_line);
	}
	if(prevGmState.zone == 2 && gmState.zone == 3) {
		return drvbState.intersect_line(Field::zone_2_to_3_line);
	}
	if(prevGmState.zone == 5 && gmState.zone == 6) {
		return drvbState.intersect_line(Field::zone_5_to_6_line);
	}
	if(prevGmState.zone == 8 && gmState.zone == 9) {
		return drvbState.intersect_line(Field::zone_8_to_9_line);
	}
#endif

#ifndef FORCE_WALL_ALIGN
	// Ir alignment
	if(prevGmState.zone == 0 && gmState.zone == 0 && abs(mt::signed_angle(mt::Vec2(0.0, 1.0), drvbState.heading)) < PI/2) {
#endif
		drvbState.heading = heading_from_ir(mt::Vec2(0.0, 1.0), currSensState);
		return drvbState;
#ifndef FORCE_WALL_ALIGN
	}
#endif

	if(prevGmState.zone == 4 && gmState.zone == 4 && abs(mt::signed_angle(mt::Vec2(0.0, -1.0), drvbState.heading) < PI/2)) {
		drvbState.heading = heading_from_ir(mt::Vec2(0.0, -1.0), currSensState);
		return drvbState;
	}

	return drvbState;
}

mt::Vec2 heading_from_ir(mt::Vec2 baseVec, SensorState const& sensState)
{
	// See fig.4
	float dist_diff = abs(sensState.backIR_dist - sensState.frontIR_dist);
	float heading_angle = asin(dist_diff/kIRSensor_apartDist);
	return mt::rotate(baseVec, heading_angle);
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