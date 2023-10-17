
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

	delay(1000);

}

void loop() 
{
	// if(whistle_detection()) {
	if(ROBUS_IsBumper(3)) {
		sensorState = get_sensors();

		p28::tie(robotState, gameState) = compute_robotGame_state(prevSensorState, sensorState, robotState, gameState);

		actionState = generate_actionState(robotState, gameState);

		hardwareState = generate_hardwareState(actionState);

		set_hardware(hardwareState);

		prevSensorState = sensorState;

		delay(kControlLoopDelay);
	}
	delay(10);
}

void buzzerFin()
{
	AX_BuzzerON(1000, 200);
	delay(400);
	AX_BuzzerON(800, 200);
	delay(400);
	AX_BuzzerON(800, 150);
	delay(200);
	AX_BuzzerON(1000, 150);
	delay(200);
	AX_BuzzerON(1500, 150);
	delay(200);
	AX_BuzzerON(240, 400);
	delay(600);
	AX_BuzzerON(120, 400);
	delay(600);
}