
#include <Arduino.h>
#include "LibRobus.h"
#include "Constants.hpp"
#include "Drivebase.hpp"
#include "Solver.hpp"
#include "WhistleDetector.hpp"
#include "ProximityDetector.hpp"
#include "TraveledPath.hpp"

void buzzerFin();

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

#ifdef AQUAMAN
	driveBase.left.pid = { 1.4, 35.5555, 0.03333333 };
	driveBase.right.pid = { 1.4, 35.5555, 0.03333333 };
#else
	driveBase.left.pid = { 1.4, 35.5555, 0.03333333 };
	driveBase.right.pid = { 1.4, 35.5555, 0.03333333 };
#endif

}

void loop() 
{
	// if(whistle_detection()) {
	if(ROBUS_IsBumper(3)) {
		driveBase = solve3(driveBase);
		// driveBase = move_to_square(driveBase, FRONT, 1, true);
		buzzerFin();
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