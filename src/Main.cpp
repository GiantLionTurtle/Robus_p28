
#include <LibRobus.h>

#include "Robot.hpp"
#include "HardwareState.hpp"

#include "Tests/UnitTests.hpp"
#include "Field.hpp"

#include "Controller.hpp"
#include "Sensors/Camera.hpp"

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
	
	it_time = Iteration_time::first();
	SensorState::init();
	delay(1000);
	

	Serial.println("Begin!");

	sensState = get_sensors();
	prevSensState = sensState;

	set_hardwareState(HardwareState::initial());

	robot.init();
	
	Serial.println("Inited the thingies");
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
	if(ROBUS_IsBumper(RIGHT)) {
		// Serial.println("sankdns");
		robot.set_target_color(controller_color);
		// Serial.println("alalla");
		bool break_ = false;
		while(true && !break_) {
			// Serial.println("sankrrrrrrdns");
			break_ = control_step();
		}
		Serial.println("boooop");
	}
	// Serial.println("out");
}

bool control_step()
{
	delay(time_to_delay_ms);
	unsigned int loop_start = millis();
	sensState = get_sensors();

	it_time = it_time.current();

	robot.update(prevSensState, sensState, it_time);
	hrdwState = hrdwState.mix(robot.generate_hardwareState(it_time));
	// print(sensState);

	// Serial.print("wheel hrwst ");
	// print(hrdwState.motors);
	// Serial.println();

	set_hardwareState(hrdwState);

	prevSensState = sensState;

	// if(ROBUS_IsBumper(RIGHT)) {
	// 	robot.cnvr.start_sequence(it_time);
	// 	ktest_color++;
	// 	robot.bin.set_bin_color(ktest_color%4);
	
	// }
	if(ROBUS_IsBumper(FRONT)) {
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