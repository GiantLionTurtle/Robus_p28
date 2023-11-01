
#include <LibRobus.h>

#include "Robot.hpp"
#include "HardwareState.hpp"

#include "UnitTests.hpp"

using namespace p28;

Robot robot;
SensorState sensState, prevSensState;
Iteration_time it_time;

GameState gmState, prevGmState;
HardwareState hrdwState;

unsigned int time_to_delay_ms;

void setup()
{
	BoardInit();
	delay(1000);

	Serial.println("Begin!");

	robot.drvb.concrete.left.pid = { 1.4, 35.5555, 0.03333333 };
	robot.drvb.concrete.right.pid = { 1.4, 35.5555, 0.03333333 };
	robot.drvb.concrete.headingPID = { 0.4, 0.18, 0.006 };

	// robot.drvb.concrete.headingPID = { 0.48, 0.18, 0.006 };

	// robot.drvb.state.heading = mt::normalize(mt::rotate(mt::Vec2(0.0, 1.0), (float)PI));
	// robot.drvb.state.heading = { 0.0, -1.0 };

	it_time = Iteration_time::first();

	sensState = get_sensors();
	gmState = GameState::initial(sensState);
	prevGmState = gmState;
	prevSensState = sensState;

	set_hardwareState(HardwareState::initial());
}

void loop()
{
	delay(40);
	it_time = it_time.current();

	if(ROBUS_IsBumper(3)) {
		while(!gmState.over) {
			delay(time_to_delay_ms);
			unsigned int loop_start = millis();
			sensState = get_sensors();
			it_time = it_time.current();
			gmState = gmState.generate_next(prevSensState, sensState, robot.drvb.state, it_time);
			robot.generate_next(prevSensState, sensState, prevGmState, gmState, it_time);
			hrdwState = hrdwState.mix(generate_hardwareState(robot));


			// Serial.println(it_time.delta_s, 4);

			set_hardwareState(hrdwState);

			prevSensState = sensState;
			prevGmState = gmState;

			if(ROBUS_IsBumper(1) || ROBUS_IsBumper(2)) {
				set_hardwareState(HardwareState());
				break;
			}
			// print(robot.drvb.state.pos);
			// Serial.print(" | ");
			// print(robot.drvb.state.heading, 4);
			// Serial.println();
			// printSensor(sensState);

			unsigned int loop_end = millis();
			unsigned int loop_duration = loop_end-loop_start;
			time_to_delay_ms = loop_duration > kControlLoopDelay ? 0 : kControlLoopDelay - (loop_duration);
		}		
	}



}