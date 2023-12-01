
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
	// drvb.headingPID = { 0.4, 0.18, 0.006 };
	drvb.headingPID = { 0.3, 0.135, 0.0045 };
	
	dumpObjective.step = DumpObjective::Done;

	cnvr.init();
}
void Robot::start_calibration()
{
	dumpObjective.step = DumpObjective::GetToDump;
	set_target_color(kRed);
	drvb.setDriveMode(Drivebase::followLine);
	waitInstruct = false;
}
void Robot::start_search()
{
	dumpObjective.step = DumpObjective::Done;
	Paths::gen_realSearchPath(drvb.pos, drvb.heading, drvb.path);
	drvb.set_path(Iteration_time::first());
	waitInstruct = false;
}
void Robot::update(SensorState prevSensState, SensorState currSensState, Iteration_time it_time)
{
	gameLogic(currSensState, prevSensState, it_time);

	if(!cnvr.in_clawMove()) {
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
	drop_zone = targetColor;
	if(drop_zone == kAllColors) {
		drop_zone = KGreen;
	}
}

void Robot::gameLogic(SensorState const& currSensState,  SensorState const& prevSensState, Iteration_time it_time)
{
	dumpObjective_helper(currSensState, it_time);
	if(currSensState.block_in_claw && (cnvr.just_dropped() || cnvr.over()) && dumpObjective.step == DumpObjective::Done) {
		
		cnvr.start_sequence(it_time);

		bin.add_block();
		if(targetColor == kAllColors) {
			bin.set_bin_color(currSensState.block_color);
			bin.close(); // Somewhy needed..
		}
	}
	
	if(dumpObjective.step == DumpObjective::Done) {
		huntLogic(currSensState, it_time);
	}
}

void Robot::dumpObjective_helper(SensorState const& currSensState, Iteration_time it_time)
{
	bool line_activ = total_activ(currSensState.lineDetector) > 3;
	if(((drvb.drvMode == Drivebase::followPath && drvb.finish) || bin.is_full()) && dumpObjective.step == DumpObjective::Done) {
		dumpObjective.step = DumpObjective::Start;
	}

	// Setup GetToLine
	if(dumpObjective.step == DumpObjective::Start) {
		Paths::gen_getToLine(drvb.pos, drvb.heading, drop_zone, drvb.path);
		drvb.set_path(it_time);
		dumpObjective.step++;
	}
	// Setup AlignToLine
	// if(dumpObjective.step == DumpObjective::Done && line_activ) {
	// 	dumpObjective.step = DumpObjective::GetToLine;
	// }
	if(dumpObjective.step == DumpObjective::GetToLine && line_activ) {
		drvb.path.reset();
		drvb.path.add_checkPoint(Paths::CheckPoint(drvb.pos, drvb.heading));
		drvb.path.add_line(kLineSensorToCenter);
		drvb.path.add_turn(mt::to_radians(60));
		drvb.path.add_turn(mt::to_radians(60));

		drvb.set_path(it_time);
		dumpObjective.step++;
	}
	// Setup GetToDump
	if(dumpObjective.step == DumpObjective::AlignToLine && (drvb.path.index > 1 && line_activ)) {
		dumpObjective.step++;
		drvb.setDriveMode(Drivebase::followLine);
	}
	// Setup DoDump
	if(dumpObjective.step == DumpObjective::GetToDump && drvb.finish && cnvr.over()) {
		drvb.pos = Field::kDumps[drop_zone];
		drvb.heading = Field::kDumpHeading[drop_zone];
		Paths::gen_drop(drvb.pos, drvb.heading, drop_zone, drvb.path);
		
		drvb.set_path(it_time);
		dumpObjective.step++;
	}
	if(dumpObjective.step == DumpObjective::DoDump && drvb.finish) {
		if(trapReleaseTimer == 0) {
			bin.release();
			trapReleaseTimer = it_time.time_ms;
		} else if((trapReleaseTimer+kOpenTrapDelay_ms) < it_time.time_ms) {
			bin.close();
			dumpObjective.step++;
			trapReleaseTimer = 0;
			waitInstruct = true;
		}
	}
}
void Robot::huntLogic(SensorState sensState, Iteration_time it_time)
{
	if(sensState.block_color != -1) { // Get into camera mode
		nFrames_noLegos = 0; // Reset object permanence
		
		// remember this position on the path if we are on the path
		if(drvb.drvMode == Drivebase::followPath) {
			if(headingMemory == mt::Vec2(0.0f)) {
				headingMemory = drvb.heading;
				posMemory = drvb.pos;
			}
			drvb.setDriveMode(Drivebase::followCam);
		}
	} else if(drvb.drvMode == Drivebase::followCam && (nFrames_noLegos++) > Tracking::kObjectPermanence) {
		
		// Gen a path to get back to the main path either to the 
		// last position that was on the path or the next checkpoint on the path
		Paths::Path path;
		
		bool backward;
		mt::Vec2 back_heading;
		mt::Vec2 heading;
		mt::Vec2 position;

		if(!drvb.path.finished() && mt::distance2(drvb.pos, posMemory) > mt::distance2(drvb.pos, drvb.path.current().targPos)) {
			position = drvb.path.current().targPos;
			back_heading = position - drvb.pos;
			heading = drvb.path.current().targHeading;
			backward = false;
		} else {
			position = posMemory;
			back_heading = drvb.pos - position;
			heading = headingMemory;
			backward = true;
		}

		if(mt::magnitude2(back_heading) < kPathFollower_headingEpsilon2) {
			back_heading = drvb.heading;
		}
		if(mt::magnitude2(heading) < kPathFollower_headingEpsilon2) {
			heading = drvb.heading;
		}
		path.add_checkPoint(Paths::CheckPoint(drvb.pos, drvb.heading, 0.0, false, 0.4, 0, 5));
		path.add_checkPoint(Paths::CheckPoint::make_turn(back_heading, 0, 5));
		path.add_checkPoint(Paths::CheckPoint(position, back_heading, 0.0, backward, 0.4, 0, 5));
		path.add_checkPoint(Paths::CheckPoint::make_turn(heading, 0, 5));

		// Erase any back to mainpath path
		for(; drvb.path.current().id == 5; ++drvb.path.index) {

		}

		Paths::hot_insert(drvb.path, path);
		Paths::deep_copy(path, drvb.path);
		drvb.set_path(it_time);
		headingMemory = mt::Vec2(0, 0);
		posMemory = mt::Vec2(0, 0);
	}
}

} // !p28