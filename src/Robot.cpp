
#include "Robot.hpp"
#include <Arduino.h>

#include "Field.hpp"
#include "LineDetector.hpp"

namespace p28 {

// Adjust the drivebase with additional info such as the position in the field
DrivebaseState adjustDrivebase(DrivebaseState drvbState, SensorState const&  currSensState, 
								GameState const&  prevGmState, GameState const&  gmState);

void ballSwerve_helper(Robot& robot, Objective obj_state);
mt::Vec2 heading_from_ir(mt::Vec2 baseVec, SensorState const& sensState);

DrivebasePath gen_test_path()
{
	DrivebasePath path;
	path.segments[0] = Pair<PathSegment, unsigned int>(
		PathSegment(mt::Vec2(1.5, -1.0), mt::Vec2(1.0, 0.0), 0.0),
		0);
	path.size = 1;

	return path;
}

DrivebasePath genpathyellowline()
{
	DrivebasePath path;
	path.segments[0] = Pair<PathSegment, unsigned int>(
		PathSegment(mt::Vec2(1.0, 1.0), mt::Vec2(1.0, 0.0), 0.6),
		0);
	path.segments[1] = Pair<PathSegment, unsigned int>(
		PathSegment(mt::Vec2(1.0, -1.0), mt::Vec2(0.0, -1.0), 0.6),
		0);
	path.segments[2] = Pair<PathSegment, unsigned int>(
		PathSegment(mt::Vec2(-1.0, -1.0), mt::Vec2(-1.0, 0.0), 0.6),
		0);
	path.segments[3] = Pair<PathSegment, unsigned int>(
		PathSegment(mt::Vec2(-1.0, 1.0), mt::Vec2(0.0, 1.0), 0.0),
		0);
	path.size = 4;

	return path;
}

void Robot::generate_next(  SensorState prevSensState, SensorState currSensState, 
				   			 GameState prevGmState, GameState gmState, Iteration_time it_time)
{
	// // New state given the new encoder data
	if(gmState.missionState.test == Objective::Start) {
		//drvb.path = gen_test_path(); 
		openArm = true;
	}
	drvb.state = drvb.state.update_kinematics(prevSensState.encoders_ticks, currSensState.encoders_ticks, it_time.delta_s);

	// Adjust drivebase with other sensors and knowledge of the game
	// drvb.state = adjustDrivebase(drvb.state, currSensState, prevGmState, gmState);
	drvb.update_path();
	drvb.update_concrete(it_time);

	// Cup zone?
	// if(gmState.missionState.knock_cup == Objective::UnderWay) {
	// 	openArm = true;
	// }

	// if(gmState.missionState.trap_ball == Objective::Start) {
	// 	ballSwerve_helper(*this, gmState.missionState.trap_ball);
	// }
}
Drivebase follow_line (Drivebase drvb)
{
	/*int numCapteur = 0;
	int numCapteur2 = 0;
	char line = get_ir_line();
	if(line >= pow(2, 7)){
		line -= pow(2, 7);
		numCapteur = 1;
	} else if (line >= pow(2, 6)){
		line -= pow(2, 6);
		numCapteur = 2;
	} else if (line >= pow(2, 5)){
		line -= pow(2, 5);
		numCapteur = 3;
	}else if (line >= pow(2, 4)){
		line -= pow(2, 4);
		numCapteur = 4;
	}else if (line >= pow(2, 3)){
		line -= pow(2, 3);
		numCapteur = 5;
	}else if (line >= pow(2, 2)){
		line -= pow(2, 2);
		numCapteur = 6;
	}else if (line >= pow(2, 1)){
		line -= pow(2, 1);
		numCapteur = 7;
	}else if (line >= pow(2, 0)){
		line -= pow(2, 0);
		numCapteur = 8;
	}
	if (line > 0){
		numCapteur2 = numCapteur +1;
	}*/
	float dir =0;
	char line = get_ir_line();
	for(int i = 0; i < 8; i++)
	{
		if((bool)(line&(1<<i)))
		{
			dir+=i-3.5;
		}
	}
	float angle = dir*5*2*PI/360;
	drvb.state.heading = mt::rotate(drvb.state.heading, -angle);
	return drvb;
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
	float heading_angle = 0.0;//asin(dist_diff/kIRSensor_apartDist);
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