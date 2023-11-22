
#include <LibRobus.h>

#include "Robot.hpp"
#include "HardwareState.hpp"

#include "Tests/UnitTests.hpp"
#include "Field.hpp"

#include "Controller.hpp"

using namespace p28;

Robot robot;
SensorState sensState, prevSensState;
Iteration_time it_time;

HardwareState hrdwState;

int ktest_color = 0;

unsigned int time_to_delay_ms;

bool control_step();

void setup()
{
	BoardInit();
	Serial1.begin(9600);
	it_time = Iteration_time::first();
	delay(1000);
	
	SensorState::init();

	Serial.println("Begin!");


	sensState = get_sensors();

	prevSensState = sensState;

	set_hardwareState(HardwareState::initial());

	robot = Robot::initial();
	// robot.start_calibration();

	// while(!ROBUS_IsBumper(3)) {}

	// it_time = it_time.current();
	// bool break_ = false;
	// while(robot.dumpObjective.step != DoDumpObjective::Done && !break_) {
	// 	break_ = control_step();
	// }
}

void loop()
{
	it_time = it_time.current();

	int controller_color = 0;//get_controller_color(Serial1);

	// if(controller_color != -1) {
	if(ROBUS_IsBumper(3)) {
		robot.set_target_color(controller_color);
		bool break_ = false;
		while(true && !break_) {
			break_ = control_step();
		}
	}
	//Serial.println("out");
}

bool control_step()
{
	delay(time_to_delay_ms);
	unsigned int loop_start = millis();
	sensState = get_sensors();

	it_time = it_time.current();
	robot.update(prevSensState, sensState, it_time);
	hrdwState = hrdwState.mix(robot.generate_hardwareState(it_time));


	set_hardwareState(hrdwState);

	prevSensState = sensState;

	if(ROBUS_IsBumper(RIGHT)) {
		// robot.cnvr.start_sequence(it_time);
		ktest_color ++ % 4;
		Bin bin;
		bin.set_bin_color(ktest_color);
	
	}
	if(ROBUS_IsBumper(0) || ROBUS_IsBumper(2)) {
		set_hardwareState(HardwareState());
		return true;
	}
	// print(robot.drvb.pos);
	//  Serial.print(" | ");
	//  print(robot.drvb.heading, 4);
	//  Serial.println();
	// print(sensState);
	// Serial.print(sensState.frontIR_dist);
	// Serial.print("'  ");
	// Serial.println(sensState.backIR_dist);


	unsigned int loop_end = millis();
	unsigned int loop_duration = loop_end-loop_start;
	time_to_delay_ms = loop_duration > kControlLoopDelay ? 0 : kControlLoopDelay - (loop_duration);
	return false;
}