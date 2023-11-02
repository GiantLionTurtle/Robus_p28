
#include "Robot.hpp"
#include <Arduino.h>
#include "Constants.hpp"
#include "Field.hpp"
#include "LineDetector.hpp"

#include "Paths.hpp"

namespace p28 {

mt::Vec2 heading_from_ir(mt::Vec2 baseVec, SensorState const& sensState, SensorState prevSensorState, mt::Vec2 fallback);

Robot Robot::initial(GameState gmState)
{
	Robot robot;
	robot.drvb.concrete.left.pid = { 1.4, 35.5555, 0.03333333 };
	robot.drvb.concrete.right.pid = { 1.4, 35.5555, 0.03333333 };
	// robot.drvb.concrete.headingPID = { 0.4, 0.18, 0.006 };
	robot.drvb.concrete.headingPID = { 0.3, 0.135, 0.0045 };

	robot.drvb.state.heading = mt::Vec2(0.0, 1.0);
	mt::Vec2 pos_init;
	if(gmState.lane == 1)
	{
		pos_init = Field::green_startPos;
	}
	else if(gmState.lane == 2)
	{
		pos_init = Field::yellow_startPos;
	} else {
		pos_init = mt::Vec2(-2, -2);
	}
	robot.drvb.state.pos = pos_init;
	return robot;
}

void Robot::generate_next(  SensorState prevSensState, SensorState currSensState, 
				   			 GameState prevGmState, GameState gmState, Iteration_time it_time)
{
	// Serial.println("YEAH");
	test_helper(gmState, it_time);
	oneTurn_helper(gmState, it_time);
	knockCup_helper(gmState, it_time);
	trapBall_helper(gmState, it_time);
	shortCut_helper(gmState, it_time);

	// New state given the new encoder data
	drvb.state = drvb.state.update_kinematics(prevSensState.encoders_ticks, currSensState.encoders_ticks, it_time.delta_s);

	// Adjust drivebase with other sensors and knowledge of the game
	adjustDrivebase(currSensState, prevSensState, prevGmState, gmState);
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

void Robot::followLine ()
{
	float dir =0;
	char line = get_ir_line();
	for(int i = 0; i < 8; i++)
	{
		if((bool)(line&(1<<i)))
		{
			dir+=i-3.5;
		}
	}
	float angle = dir*2*PI/360;
	drvb.state.heading = mt::normalize(mt::rotate(drvb.path.current().targHeading, -angle));
}

void Robot::adjustDrivebase(SensorState const& currSensState,  SensorState const& prevSensState,
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
			b. Same as with color change, intersect with line*/
			if((gmState.zone == 6 || gmState.zone == 7 || gmState.zone == 8) && ! gmState.missions.trap_ball.underway())
			{
				followLine();
			}

			if(prevGmState.zone == 8 && gmState.zone == 9)
			{
				drvb.state.pos = mt::Vec2(2.0/3.281, 4.0/3.281);
				drvb.state.heading = normalize(mt::Vec2(-1., 1.0));
			}
		
		/*3. Bumpers in shortcut
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
		drvb.state.pos = Field::zone_1_to_2_line.offset(-drvb.state.heading*kColorSensorToCenter).closest_point(drvb.state.pos);
	}
	if(prevGmState.zone == 2 && gmState.zone == 3) {
		drvb.state.pos = Field::zone_2_to_3_line.offset(-drvb.state.heading*kColorSensorToCenter).closest_point(drvb.state.pos);
	}
	// if(prevGmState.zone == 5 && gmState.zone == 6) {
	// 	drvb.state.pos = Field::zone_5_to_6_line.offset(-drvb.state.heading*kColorSensorToCenter).closest_point(drvb.state.pos);;
	// }

#endif

// #ifndef FORCE_WALL_ALIGN
// 	// Ir alignment
// 	if(prevGmState.zone == 0 && gmState.zone == 0 && abs(mt::signed_angle(mt::Vec2(0.0, 1.0), drvb.state.heading)) < PI/2) {
// #endif
// 		drvb.state.heading = heading_from_ir(mt::Vec2(0.0, 1.0), currSensState, drvb.state.heading);
// #ifndef FORCE_WALL_ALIGN
// 	}
// #endif

// 	if(prevGmState.zone == 4 && gmState.zone == 4 && abs(mt::signed_angle(mt::Vec2(0.0, -1.0), drvb.state.heading) < PI/2)) {
// 		drvb.state.heading = heading_from_ir(mt::Vec2(0.0, -1.0), currSensState, drvb.state.heading);
// 	}
}

mt::Vec2 heading_from_ir(mt::Vec2 baseVec, SensorState const& sensState, SensorState prevSensorState, mt::Vec2 fallback)
{
	if (sensState.frontIR_dist > 500 || sensState.backIR_dist > 500)
		return fallback;
	// See fig.4

	// Add Previous sensorstate to make an everage???
	float dist_diff = sensState.backIR_dist - sensState.frontIR_dist;
	float prev_diff = prevSensorState.backIR_dist - prevSensorState.frontIR_dist;
	dist_diff = (dist_diff*3+prev_diff)/4;
	if(abs(dist_diff) >= kIRSensor_apartDist || abs(prev_diff) >= kIRSensor_apartDist)

		return fallback;
	float heading_angle = mt::clamp(-asin(dist_diff/kIRSensor_apartDist), mt::to_radians(-15), mt::to_radians(15));
	//  Serial.println(dist_diff);
	//  Serial.println(prev_diff);
	
	return mt::normalize(mt::rotate(baseVec, heading_angle/3));
}

} // !p28