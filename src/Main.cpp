
#include <LibRobus.h>

#include "Robot.hpp"
#include "HardwareState.hpp"

#include "Tests/UnitTests.hpp"
#include "Field.hpp"

using namespace p28;

Robot robot;
SensorState sensState, prevSensState;
Iteration_time it_time;

HardwareState hrdwState;

unsigned int time_to_delay_ms;

void setup()
{
	BoardInit();
	delay(1000);
	
	SensorState::init();

	Serial.println("Begin!");


	//robot.drvb.state.pos = {2.286 , 3.64};
	//robot.drvb.state.pos = Field::green_startPos;
	// robot.drvb.state.pos  = mt::Vec2(2.701 , 1.2192 - 0.16);
	//robot.drvb.state.heading = {0.0, -1.0};

	// robot.drvb.concrete.headingPID = { 0.48, 0.18, 0.006 };

	// robot.drvb.state.heading = mt::normalize(mt::rotate(mt::Vec2(0.0, 1.0), (float)PI));
	// robot.drvb.state.heading = { 0.0, -1.0 };

	it_time = Iteration_time::first();

	sensState = get_sensors();

	prevSensState = sensState;

	set_hardwareState(HardwareState::initial());
	// Tests::arc_generation();
}

void loop()
{
	it_time = it_time.current();

	if(ROBUS_IsBumper(3)) {
		robot = Robot::initial();
		while(true) {
			delay(time_to_delay_ms);
			unsigned int loop_start = millis();
			sensState = get_sensors();
			// print(sensState);
			it_time = it_time.current();
			robot.generate_next(prevSensState, sensState, it_time);
			hrdwState = hrdwState.mix(generate_hardwareState(robot));

			// Serial.println(it_time.delta_s, 4);

			set_hardwareState(hrdwState);

			prevSensState = sensState;

			if(ROBUS_IsBumper(0) || ROBUS_IsBumper(1) || ROBUS_IsBumper(2)) {
				set_hardwareState(HardwareState());
				break;
			}
			// print(robot.drvb.state.pos);
			// Serial.print(" | ");
			//   print(robot.drvb.state.heading, 4);
			//  Serial.println();
			 //print(sensState);
			// Serial.print(sensState.frontIR_dist);
			// Serial.print("'  ");
			// Serial.println(sensState.backIR_dist);
		

			unsigned int loop_end = millis();
			unsigned int loop_duration = loop_end-loop_start;
			time_to_delay_ms = loop_duration > kControlLoopDelay ? 0 : kControlLoopDelay - (loop_duration);
		}		
	}
}