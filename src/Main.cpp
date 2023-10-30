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
#include "TestPID.hpp"

using namespace p28;

// SensorState sensorState;
// SensorState prevSensorState;

// Robot robot;
// GameState gameState;
// GameState prevGameState;

// HardwareState hardwareState;

// Iteration_time it_time;

void buzzerFin();
/*
void setup()
{
	BoardInit();
	delay(1000);
	Serial.println("Begin!");

	robot.drvb.concrete.left.pid = { 1.4, 35.5555, 0.03333333 };
	robot.drvb.concrete.right.pid = { 1.4, 35.5555, 0.03333333 };
	robot.drvb.concrete.headingPID = { 0.5, 0.0, 0.0 };

	init_detector();
	init_whistle();
	init_color_sensor();

	// // delay(1000);
	// while(!ROBUS_IsBumper(3)) { delay(50); }

	// // TestPID::Ziegler_Nichols(1.8, 0.3);
	// PID pid { 1.4, 35.5555, 0.03333333 };
	// TestPID::test_pid_straightLine(pid, pid, 0.3);

	it_time = it_time.current();

	SensorState sensState = get_sensors();
	// gameState = GameState::initial(sensState);
	// robot.generate_next(sensorState, sensorState, prevGameState, gameState, it_time);
	prevSensorState = sensorState;

	// Tests::vector_maths();
	// Tests::forward_kinematics();
	// Tests::acceleration_profile();
	// Tests::arc_generation();

	// Tests::vectors();
	// Tests::near_equality();
	// Tests::test_pid_straightLine(robot.drvb.concrete.left.pid, robot.drvb.concrete.left.pid, 0.5);
}
*/

/*
void loop() 
{
	// delay(500);
	// sensorState = get_sensors();
	// printSensor(sensorState);
	it_time = it_time.current();

	// if(whistle_detection()) {
	if(ROBUS_IsBumper(3)) {
		while(!gameState.over) {
			// Pause for a bit to allow everything to catch up 
			delay(kControlLoopDelay);

			// No data is fed, this is a read function
			sensorState = get_sensors();
			it_time = it_time.current();

			gameState = prevGameState.generate_next(prevSensorState, sensorState, robot.drvb.state, it_time);
			robot.generate_next(prevSensorState, sensorState, prevGameState, gameState, it_time);

			// Create the data to send to the hardware
			// the robot is fed and outputed to keep track
			// of the motors and follow paths
			hardwareState = generate_hardwareState(robot);

			// // Only processed data is fed, it is a write function
			set_hardwareState(hardwareState);

			// // Keep the previous sensor and game states for useful deltas
			prevSensorState = sensorState;
			prevGameState = gameState;
		}
		Serial.println("out");
	}

	// Serial.println(static_cast<int>(get_color()));
	// delay(100);
}
*/