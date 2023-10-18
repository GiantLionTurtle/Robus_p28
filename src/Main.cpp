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
#include "ActionState.hpp"
#include "HardwareState.hpp"
#include "sensors.hpp"

using namespace p28;

SensorState sensorState;
SensorState prevSensorState;

Robot robot;
GameState gameState;
GameState prevGameState;

ActionState actionState;
HardwareState hardwareState;

void buzzerFin();

void setup()
{
	BoardInit();
	Serial.println("Begin!");

	init_detector();
	init_whistle();
	init_color_sensor();

	delay(1000);

}

void loop() 
{
	// if(whistle_detection()) {
	if(ROBUS_IsBumper(3)) {
		while(!gameState.over) {
			// No data is fed, this is a read function
			sensorState = get_sensors();

			gameState = prevGameState.next(prevSensorState, sensorState);
			robot = robot.next(prevSensorState, sensorState, prevGameState, gameState);

			// Robot is fed to ensure the generated actions
			// are aligned with the physical reality of the robot
			actionState = generate_actionState(actionState, robot, gameState);

			// Create the data to send to the hardware
			// the robot is fed and outputed to keep track
			// of the motors and follow paths
			tie(hardwareState, robot) = generate_hardwareState(actionState, robot);

			// Only processed data is fed, it is a write function
			set_hardwareState(hardwareState);

			// Keep the previous sensor and game states for useful deltas
			prevSensorState = sensorState;
			prevGameState = gameState;

			// Pause for a bit to allow everything to catch up 
			delay(kControlLoopDelay);
		}
	}
	Serial.println(static_cast<int>(get_color()));
	delay(100);
}