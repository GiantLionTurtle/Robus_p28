
#include "Robot.hpp"
#include <Arduino.h>
#include "Constants.hpp"
#include "Field.hpp"
#include "LineDetector.hpp"

#include "Paths.hpp"

namespace p28 {

// Adjust the drivebase with additional info such as the position in the field
DrivebaseState adjustDrivebase(DrivebaseState drvbState, SensorState const&  currSensState, 
								GameState const&  prevGmState, GameState const&  gmState);

mt::Vec2 heading_from_ir(mt::Vec2 baseVec, SensorState const& sensState, mt::Vec2 fallback);
Drivebase follow_line (Drivebase drvb);


void Robot::generate_next(  SensorState prevSensState, SensorState currSensState, 
				   			 GameState prevGmState, GameState gmState, Iteration_time it_time)
{
	test_helper(gmState, it_time);
	oneTurn_helper(gmState, it_time);
	knockCup_helper(gmState, it_time);
	trapBall_helper(gmState, it_time);
	shortCut_helper(gmState, it_time);

	// New state given the new encoder data
	drvb.state = drvb.state.update_kinematics(prevSensState.encoders_ticks, currSensState.encoders_ticks, it_time.delta_s);

	// Adjust drivebase with other sensors and knowledge of the game
	drvb.state = adjustDrivebase(drvb.state, currSensState, prevGmState, gmState);
	drvb.update_path(it_time);
	drvb.update_concrete(it_time);
}

void Robot::test_helper(GameState gmState, Iteration_time it_time)
{
	// Test mission
	if(gmState.missions.test.start()) {
		drvb.set_path(Paths::gen_test(), it_time);
	}
}
void Robot::knockCup_helper(GameState gmState, Iteration_time it_time)
{
	// Knock cup mission
	if(gmState.missions.knock_cup.start()) {
		openArm = true;

		// Modify path if we are in the green lane (turn to knock the cup)
		if(gmState.lane == 1) {
			drvb.set_path(Paths::add_greenLaneKnockCup(drvb.path), it_time);
		}
	} else if(gmState.missions.knock_cup.done()) {
		openArm = false;
	}
}
void Robot::trapBall_helper(GameState gmState, Iteration_time it_time)
{
	// Trap ball mission
	if(gmState.missions.trap_ball.start()) {
		drvb.set_path(Paths::add_pingPong(drvb.path, drvb.state), it_time);
	}
	if(gmState.missions.trap_ball.underway() && drvb.path.index == kCupRelease_pathIndex) {
		releaseCup = true;
	} 
	if(gmState.missions.trap_ball.done()) {
		releaseCup = false;
	}
}
void Robot::oneTurn_helper(GameState gmState, Iteration_time it_time)
{
	// One turn mission
	if(gmState.missions.one_turn.start()) {
		if(gmState.lane == 1) {
			drvb.set_path(Paths::gen_greenLane(), it_time);
		} else if(gmState.lane == 2) {
			drvb.set_path(Paths::gen_yellowLane(), it_time);
		}
	}
}
void Robot::shortCut_helper(GameState gmState, Iteration_time it_time)
{
	// Shortcut mission
	if(gmState.missions.one_shortcut_turn.start()) {
		drvb.set_path(Paths::gen_shortcut(), it_time);
	}
}

void followLine (Drivebase drvb)
{
	
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
		drvbState.heading = heading_from_ir(mt::Vec2(0.0, 1.0), currSensState, drvbState.heading);
		return drvbState;
#ifndef FORCE_WALL_ALIGN
	}
#endif

	if(prevGmState.zone == 4 && gmState.zone == 4 && abs(mt::signed_angle(mt::Vec2(0.0, -1.0), drvbState.heading) < PI/2)) {
		drvbState.heading = heading_from_ir(mt::Vec2(0.0, -1.0), currSensState, drvbState.heading);
		return drvbState;
	}

	return drvbState;
}

mt::Vec2 heading_from_ir(mt::Vec2 baseVec, SensorState const& sensState, mt::Vec2 fallback)
{
	// See fig.4

	// Add Previous sensorstate to make an everage???
	float dist_diff = sensState.backIR_dist - sensState.frontIR_dist;
	if(dist_diff >= kIRSensor_apartDist)
		return fallback;
	float heading_angle = asin(dist_diff/kIRSensor_apartDist);
	// Serial.println(dist_diff);
	return mt::normalize(mt::rotate(baseVec, heading_angle));
}

} // !p28