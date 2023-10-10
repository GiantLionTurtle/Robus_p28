
#include <Arduino.h>
#include "LibRobus.h"
#include "Constants.hpp"
#include "Drivebase.hpp"
#include "Solver.hpp"
#include "WhistleDetector.hpp"
#include "ProximityDetector.hpp"
#include "TraveledPath.hpp"

Drivebase driveBase;
bool start = false;

void setup()
{
	BoardInit();
	Serial.println("Begin!");

	init_detector();
	init_whistle();
	init_legalityMatrix();
	init_path();
	// buzzerFin();

	delay(1000);
	driveBase.left.ID = LEFT;
	driveBase.right.ID = RIGHT;

#ifdef ROBOT_A
	driveBase.left.pid = { 1.4, 35.5555, 0.03333333 };
	driveBase.right.pid = { 1.4, 35.5555, 0.03333333 };
#else
	driveBase.left.pid = { 2.8, 53.4, 0.055 };
	driveBase.right.pid = { 2.8, 53.4, 0.055 };
#endif

}

void loop() 
{
	if(ROBUS_IsBumper(3)) {
		driveBase = solve3(driveBase);
	}
}