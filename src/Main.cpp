#include <LibRobus.h>
#include <ColorSensor.hpp>

#include <Arduino.h>
#include "LibRobus.h"
#include "Constants.hpp"
#include "Drivebase.hpp"
#include "WhistleDetector.hpp"
#include "ProximityDetector.hpp"
#include "TraveledPath.hpp"

#include "RobotState.hpp"
#include "GameState.hpp"
#include "ActionState.hpp"
#include "HardwareState.hpp"
#include "sensors.hpp"

SensorState sensorState;
SensorState prevSensorState;

RobotState robotState;
GameState gameState;

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
	/*if(ROBUS_IsBumper(3)) {
		sensorState = get_sensors();

		// p28::tie(robotState, gameState) = compute_robotGame_state(prevSensorState, sensorState, robotState, gameState);

		// actionState = generate_actionState(robotState, gameState);

		hardwareState = generate_hardwareState(actionState);

		set_hardwareState(hardwareState);

		prevSensorState = sensorState;

		delay(kControlLoopDelay);
	}*/
	Serial.println(get_color());
	delay(100);
}