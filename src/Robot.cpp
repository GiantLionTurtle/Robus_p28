
#include "Robot.hpp"
#include <Arduino.h>
#include "Constants.hpp"
#include "Field.hpp"
#include "Sensors/LineDetector.hpp"
#include "Controller.hpp"


#include "Paths.hpp"
#include "Subsystems/Bin.hpp"

namespace p28 {


Robot Robot::initial()
{
	Robot robot;
	Conveyor cnvr; //testconveyor
	Iteration_time it_time = Iteration_time::first();  //test conveyor
	robot.drvb.leftWheel.pid = { 1.4, 35.5555, 0.03333333 };
	robot.drvb.rightWheel.pid = { 1.4, 35.5555, 0.03333333 };
	// robot.drvb.concrete.headingPID = { 0.4, 0.18, 0.006 };
	robot.drvb.headingPID = { 0.3, 0.135, 0.0045 };
	robot.cnvr = cnvr.Start_ConveyorTime(it_time,cnvr); //test
	
	robot.drvb.pos = Field::kDumps[0];
	robot.drvb.heading = Field::kDumpHeading[0];

	//robot.drvb.set_path(Paths::gen_test(), Iteration_time::first());
	robot.drvb.set_path(Paths::gen_drop(Field::kDumps[0], Field::kDumpHeading[0], 0), Iteration_time::first());
	return robot;
}
void Robot::start_calibration()
{
	dumpObjective.step = DoDumpObjective::GetToDump;
	targetColor = kRed;
	drvb.setDriveMode(Drivebase::followLine);
}
void Robot::update(  SensorState prevSensState, SensorState currSensState, Iteration_time it_time)
{
	gameLogic(currSensState, prevSensState, it_time);
	drvb.update(currSensState, prevSensState, it_time);
}
HardwareState Robot::generate_hardwareState(Iteration_time it_time)
{
	HardwareState hrdwState;
	Conveyor cnvrState;

	//hrdwState = drvb.aggregate(hrdwState);
	hrdwState = bin.Aggregate_hardwareState(hrdwState);
	hrdwState = cnvr.update_conveyorSequence(cnvrState, hrdwState, it_time);
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
		drvb.set_path(Paths::gen_getToLine(drvb.pos, drvb.heading, internalColor), it_time);
		dumpObjective.step++;
	}
	if(dumpObjective.step == DoDumpObjective::GetToLine && drvb.finish) {
		drvb.setDriveMode(Drivebase::followLine);
		dumpObjective.step++;
	}
	if(dumpObjective.step == DoDumpObjective::GetToDump && drvb.finish) {
		drvb.pos = Field::kDumps[internalColor];
		drvb.heading = Field::kDumps[internalColor];
		drvb.set_path(Paths::gen_drop(drvb.pos, drvb.heading, internalColor), it_time);
		dumpObjective.step++;
	}
	// if(dumpObjective.step == DoDumpObjective::DoDump && drvb.move_id() == kDumpPointId) {
	// 	bin.release();
	// }
	if(dumpObjective.step == DoDumpObjective::DoDump && drvb.finish) {
		// bin.close();
		dumpObjective.step++;
	}
}

} // !p28