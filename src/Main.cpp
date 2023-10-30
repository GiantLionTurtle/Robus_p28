
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

PID pid = { 1.4, 35.5555, 0.03333333 };
// PID pid = { 4, 0.0, 0.0 };
Error error;

void setup()
{
	BoardInit();
	delay(1000);

	Serial.println("Begin!");

	robot.drvb.concrete.left.pid = { 1.4, 35.5555, 0.03333333 };
	robot.drvb.concrete.right.pid = { 1.4, 35.5555, 0.03333333 };
	robot.drvb.concrete.headingPID = { 0.1, 1.0, 0.002 };

	it_time = Iteration_time::first();

	sensState = get_sensors();
	gmState = GameState::initial(sensState);
	prevGmState = gmState;
	prevSensState = sensState;
}
float buffer_mult = 0.9;
void loop()
{
	delay(40);
	it_time = it_time.current();

	if(ROBUS_IsBumper(3)) {
		while(!gmState.over) {
			delay(kControlLoopDelay);

			it_time = it_time.current();
			sensState = get_sensors();

			gmState = gmState.generate_next(prevSensState, sensState, robot.drvb.state, it_time);
			robot.generate_next(prevSensState, sensState, prevGmState, gmState, it_time);
			hrdwState = hrdwState.mix(generate_hardwareState(robot));


			// print(robot.drvb.state.wheelsVelocities);
			// Serial.print(" | ");
			print(robot.drvb.state.pos);
			Serial.print(" | ");
			print(robot.drvb.state.heading);

			Serial.print("\n");
			// Serial.println(it_time.delta_s, 4);


			set_hardwareState(hrdwState);

			prevSensState = sensState;
			prevGmState = gmState;

			if(ROBUS_IsBumper(2)) {
				set_hardwareState(HardwareState());
				break;
			}
		}		
	}



}