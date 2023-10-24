#include <LibRobus.h>
#include <ColorSensor.hpp>

#include <Arduino.h>
#include "LibRobus.h"
#include "Constants.hpp"
#include "Drivebase.hpp"
#include "WhistleDetector.hpp"
#include "ProximityDetector.hpp"

#include "Robot.hpp"
#include "GameState.hpp"
#include "HardwareState.hpp"
#include "sensors.hpp"

#include "TestPID.hpp"
#include "Iteration_time.hpp"

#include "UnitTests.hpp"

using namespace p28;

SensorState sensorState;
SensorState prevSensorState;

Robot robot;
GameState gameState;
GameState prevGameState;

HardwareState hardwareState;

Iteration_time it_time;

void buzzerFin();

void setup()
{
	BoardInit();
	delay(1000);
	Serial.println("Begin!");

	robot.drvb.concrete.left.pid = { 1.4, 35.5555, 0.03333333 };
	robot.drvb.concrete.right.pid = { 1.4, 35.5555, 0.03333333 };

	init_detector();
	init_whistle();
	init_color_sensor();

	// // delay(1000);
	// while(!ROBUS_IsBumper(3)) { delay(50); }

	// // TestPID::Ziegler_Nichols(1.8, 0.3);
	// PID pid { 1.4, 35.5555, 0.03333333 };
	// TestPID::test_pid_straightLine(pid, pid, 0.3);

	it_time = it_time.current();

	// Tests::vector_maths();
}

void loop() 
{
	// if(whistle_detection()) {
	if(ROBUS_IsBumper(3)) {
		while(!gameState.over) {
			delay(kControlLoopDelay);

			// No data is fed, this is a read function
			sensorState = get_sensors();
			it_time = it_time.current();

			gameState = prevGameState.generate_next(prevSensorState, sensorState);
			robot = robot.generate_next(prevSensorState, sensorState, prevGameState, gameState, it_time);

			// Create the data to send to the hardware
			// the robot is fed and outputed to keep track
			// of the motors and follow paths
			hardwareState = generate_hardwareState(robot);

			// Only processed data is fed, it is a write function
			set_hardwareState(hardwareState);

			// Keep the previous sensor and game states for useful deltas
			prevSensorState = sensorState;
			prevGameState = gameState;

			// Pause for a bit to allow everything to catch up 
		}
	}
	// Serial.println(static_cast<int>(get_color()));
	// delay(100);
}