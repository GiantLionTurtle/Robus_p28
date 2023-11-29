
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
	
	// drvb.pos = Field::kDumps[0];
	// drvb.heading = Field::kDumpHeading[0];

	//robot.drvb.set_path(Paths::gen_test(), Iteration_time::first());
	// robot.drvb.set_path(Paths::gen_drop(Field::kDumps[0], Field::kDumpHeading[0], 0), Iteration_time::first());
	// drvb.pos = { 0.0 };
	// drvb.heading = { 0.0, 1.0 };
	// drvb.set_path(Iteration_time::first());
	dumpObjective.step = DumpObjective::Done;

	cnvr.init();
}
void Robot::start_calibration()
{
	dumpObjective.step = DumpObjective::GetToDump;
	set_target_color(kRed);
	drvb.setDriveMode(Drivebase::followLine);
	// nBlocksInCycle = 5;
	waitInstruct = false;
}
void Robot::start_search()
{
	dumpObjective.step = DumpObjective::Done;
	Paths::gen_realSearchPath(drvb.pos, drvb.heading, drvb.path);
	drvb.set_path(Iteration_time::first());
	inHunt = true;
	waitInstruct = false;
}
void Robot::update(SensorState prevSensState, SensorState currSensState, Iteration_time it_time)
{
	gameLogic(currSensState, prevSensState, it_time);
	// Serial.println("upd");

	// if(/*!cnvr.just_dropped()||*/ cnvr.over()) {
		// Serial.print("Drivemode ");
		// Serial.println(drvb.drvMode);
		drvb.update(currSensState, prevSensState, it_time);
	// } else {
		// drvb.zero(currSensState, prevSensState, it_time);
	// }
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
	drop_zone = targetColor;
	if(drop_zone == kAllColors) {
		drop_zone = kRed;
	}

}

void Robot::gameLogic(SensorState const& currSensState,  SensorState const& prevSensState, Iteration_time it_time)
{
	dumpObjective_helper(currSensState, it_time);
	if(currSensState.block_in_claw && (cnvr.just_dropped() || cnvr.over())) {
		
		cnvr.start_sequence(it_time);

		bin.add_block();
		nBlocksInCycle++;
		if(targetColor == kAllColors) {
			// Serial.print("Found block color ");
			// Serial.print(currSensState.block_color);
			// Serial.print(" :: ");
			// Serial.println(prevSensState.block_color);
			bin.set_bin_color(currSensState.block_color);
			bin.close(); // Should not be needed?
		}
	}
	
	if(dumpObjective.step == DumpObjective::Done) {
		huntLogic(currSensState, it_time);
	}
	//else {
	// 		Paths::gen_searchPath(drvb.pos, drvb.heading, drvb.path);
	// 		drvb.set_path(it_time);
	// 		nBlocksInCycle = 0;
	// 	}
	// }

}

void Robot::dumpObjective_helper(SensorState const& currSensState, Iteration_time it_time)
{
	// Serial.print("Dump obj start ");
	// Serial.print(dumpObjective.step);
	// Serial.print(" :: ");
	// Serial.print(drvb.drvMode);
	// Serial.print(" :: ");
	// Serial.println(drvb.finish);
	if(drvb.drvMode == Drivebase::followPath && drvb.finish && dumpObjective.step == DumpObjective::Done)
	{
		// dumpObjective.step = DumpObjective::Start;
		// Serial.println("Staaaaaart");
	}

	if(dumpObjective.step == DumpObjective::Start) {
		Paths::gen_getToLine(drvb.pos, drvb.heading, drop_zone, drvb.path);
		drvb.set_path(it_time);
		dumpObjective.step++;
	}
	if(dumpObjective.step == DumpObjective::GetToLine && (drvb.drvMode == Drivebase::followPath && (drvb.finish || total(currSensState.lineDetector)>4))) {
		drvb.path.reset();
		drvb.path.add_checkPoint(Paths::CheckPoint(drvb.pos, drvb.heading));
		drvb.path.add_line(kLineSensorToCenter);
		drvb.path.add_turn(mt::to_radians(30));
		drvb.path.add_turn(mt::to_radians(60));
		drvb.set_path(it_time);
		Serial.println("Done get to line!");
		dumpObjective.step++;
		alignToLineTimer = it_time.time_ms;
	}
	if(dumpObjective.step == DumpObjective::AlignToLine && (drvb.finish 
			|| (total(currSensState.lineDetector) > 4 && drvb.path.index > 2 && alignToLineTimer+500 < it_time.time_ms))) {
		Serial.print("Done align! ");
		Serial.print(total(currSensState.lineDetector));
		Serial.print(" :: ");
		Serial.println(drvb.path.index);
		dumpObjective.step++;
		drvb.setDriveMode(Drivebase::followLine);
		alignToLineTimer = 0;
	}
	if(dumpObjective.step == DumpObjective::GetToDump && drvb.finish) {
		drvb.pos = Field::kDumps[drop_zone];
		drvb.heading = Field::kDumpHeading[drop_zone];
		Paths::gen_drop(drvb.pos, drvb.heading, drop_zone, drvb.path);
		
		drvb.set_path(it_time);
		dumpObjective.step++;
	}
	if(dumpObjective.step == DumpObjective::DoDump && drvb.finish) {
		// Serial.println("Finish dump! ");

		if(trapReleaseTimer == 0) {
			bin.release();
			trapReleaseTimer = it_time.time_ms;
		} else if((trapReleaseTimer+2000) < it_time.time_ms) {
			bin.close();
			dumpObjective.step++;
			trapReleaseTimer = 0;
			waitInstruct = true;
		}
	}
	// Serial.print("Dump obj ");
	// Serial.println(dumpObjective.step);
}
void Robot::huntLogic(SensorState sensState, Iteration_time it_time)
{
	if(sensState.block_color != -1) {
		nFrames_noLegos = 0;
		if(drvb.drvMode == Drivebase::followPath) {
			if(headingMemory == mt::Vec2(0.0f)) {
				headingMemory = drvb.heading;
				posMemory = drvb.pos;
			} else {
				drvb.path.index = 3; // It takes three checkpoints to get back to path
			}
			drvb.setDriveMode(Drivebase::followCam);
		}
	} else if(drvb.drvMode == Drivebase::followCam && (nFrames_noLegos++) > 16) {
		Paths::Path path;
		
		bool backward;
		mt::Vec2 heading;

		if(!drvb.path.finished() && mt::distance2(drvb.pos, posMemory) > mt::distance2(drvb.pos, drvb.path.current().targPos)) {
			heading = drvb.path.current().targPos - drvb.pos;
			backward = false;
		} else {
			 heading = drvb.pos - posMemory;
			 backward = true;
		}

		if(mt::magnitude2(heading) < kPathFollower_headingEpsilon2) {
			heading = drvb.heading;
		}
		path.add_checkPoint(Paths::CheckPoint::make_turn(heading));

		path.add_checkPoint(Paths::CheckPoint(posMemory, heading, 0.0, backward));
		path.add_checkPoint(Paths::CheckPoint::make_turn(headingMemory));
\

		Paths::hot_insert(drvb.path, path);
		Paths::deep_copy(path, drvb.path);
		drvb.set_path(it_time);
		headingMemory = mt::Vec2(0, 0);
		posMemory = mt::Vec2(0, 0);
	}
}

} // !p28