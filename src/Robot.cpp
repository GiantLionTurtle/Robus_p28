
#include "Robot.hpp"
#include <Arduino.h>
#include "Constants.hpp"
#include "Field.hpp"
#include "Sensors/LineDetector.hpp"
#include "Controller.hpp"


#include "Paths.hpp"
#include "Subsystems/Bin.hpp"

namespace p28 {


void Robot::init()
{
	drvb.leftWheel.pid = { 1.4, 35.5555, 0.03333333 };
	drvb.rightWheel.pid = { 1.4, 35.5555, 0.03333333 };
	// robot.drvb.concrete.headingPID = { 0.4, 0.18, 0.006 };
	drvb.headingPID = { 0.3, 0.135, 0.0045 };
	
	drvb.pos = Field::kDumps[0];
	drvb.heading = Field::kDumpHeading[0];

	//robot.drvb.set_path(Paths::gen_test(), Iteration_time::first());
	// robot.drvb.set_path(Paths::gen_drop(Field::kDumps[0], Field::kDumpHeading[0], 0), Iteration_time::first());
	drvb.setDriveMode(Drivebase::followPath);
	cnvr.init();
}
void Robot::start_calibration()
{
	dumpObjective.step = DoDumpObjective::GetToDump;
	targetColor = kRed;
	drvb.setDriveMode(Drivebase::followLine);
}
void Robot::update(SensorState prevSensState, SensorState currSensState, Iteration_time it_time)
{
	gameLogic(currSensState, prevSensState, it_time);
	// Serial.println("upd");

	if(/*!cnvr.climbing() || */cnvr.over()) {
		// Serial.print("Drivemode ");
		// Serial.println(drvb.drvMode);
		drvb.update(currSensState, prevSensState, it_time);
	} else {
		drvb.zero(currSensState, prevSensState, it_time);
	}
	cnvr.update(it_time);
}
HardwareState Robot::generate_hardwareState(Iteration_time it_time)
{
	HardwareState hrdwState;

	hrdwState = drvb.aggregate(hrdwState);
	hrdwState = bin.aggregate(hrdwState);
	hrdwState = cnvr.aggregate(hrdwState);
	return hrdwState;
}
void Robot::set_target_color(int controller_color)
{
	targetColor = controller_color;
	bin.set_bin_color(targetColor);
}

void Robot::gameLogic(SensorState const& currSensState,  SensorState const& prevSensState, Iteration_time it_time)
{
	if(bin.is_full() && dumpObjective.step == DoDumpObjective::Done) {
		dumpObjective.step = DoDumpObjective::Start;
	}

	int internalColor = targetColor;
	if(internalColor == KAll) {
		internalColor = kRed;
	}

	if(dumpObjective.step == DoDumpObjective::Start) {
		Paths::gen_getToLine(drvb.pos, drvb.heading, internalColor, drvb.path);
		drvb.set_path(it_time);
		dumpObjective.step++;
	}
	if(dumpObjective.step == DoDumpObjective::GetToLine && drvb.finish) {
		drvb.setDriveMode(Drivebase::followLine);
		dumpObjective.step++;
	}
	if(dumpObjective.step == DoDumpObjective::GetToDump && drvb.finish) {
		drvb.pos = Field::kDumps[internalColor];
		drvb.heading = Field::kDumps[internalColor];
		Paths::gen_drop(drvb.pos, drvb.heading, internalColor, drvb.path);
		drvb.set_path(it_time);
		dumpObjective.step++;
	}
	// if(dumpObjective.step == DoDumpObjective::DoDump && drvb.move_id() == kDumpPointId) {
	// 	bin.release();
	// }
	if(dumpObjective.step == DoDumpObjective::DoDump && drvb.finish) {
		// bin.close();
		dumpObjective.step++;
	}

	if(currSensState.block_in_claw/* && !cnvr.climbing()*/ && cnvr.over()) {
		cnvr.start_sequence(it_time);
	}
	
	if(dumpObjective.step == DoDumpObjective::Done) {
		huntLogic(currSensState, it_time);
	}
	if(dumpObjective.step == DoDumpObjective::Done && drvb.path.index == drvb.path.size && drvb.drvMode == Drivebase::followPath) {
		Paths::gen_searchPath(drvb.pos, drvb.heading, drvb.path);
		drvb.set_path(it_time);
	}
}
void Robot::huntLogic(SensorState sensState, Iteration_time it_time)
{
	if(sensState.block_offset != mt::i32Vec2(0, 0)) {
		headingMemory = drvb.heading;
		posMemory = drvb.pos;
		drvb.setDriveMode(Drivebase::followCam);
		nFrames_noLegos = 0;
		
	} else if(headingMemory != mt::Vec2(0, 0) && (nFrames_noLegos++) > 16) {
		// Serial.println("Baack");
		Paths::Path path;
		mt::Vec2 backHeading = drvb.pos - posMemory;
		if(mt::magnitude2(backHeading) < kPathFollower_headingEpsilon2) {
			backHeading = drvb.heading;
		}
		// print(backHeading);
		// Serial.print(" :: ");
		// println(posMemory);
		path.add_checkPoint(Paths::CheckPoint::make_turn (backHeading));
		path.add_checkPoint(Paths::CheckPoint (posMemory, backHeading, 0.0, true));
		path.add_checkPoint(Paths::CheckPoint::make_turn (headingMemory));
		Paths::hot_insert(drvb.path, path);
		drvb.path = path;
		drvb.set_path(it_time);
		headingMemory = mt::Vec2(0, 0);
		posMemory = mt::Vec2(0, 0);
	}
}

} // !p28