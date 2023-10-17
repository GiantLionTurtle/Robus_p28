
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

void init_drivebase()
{
	driveBase = Drivebase{};
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

}

void loop() 
{
	if(whistle_detection()) {
	// if(ROBUS_IsBumper(3)) {
		init_drivebase();
		driveBase = solve3(driveBase);
		// driveBase = move_to_square(driveBase, FRONT, 1, true);
		driveBase = move_to_square(driveBase, FRONT, 1);
		delay(kDecelerationDelay);
		if(driveBase.sq_x == 2) {
			driveBase = move_to_square(driveBase, LEFT, 1);
		}
		delay(kDecelerationDelay);
		if(driveBase.sq_x == 0) {
			driveBase = move_to_square(driveBase, RIGHT, 1);
		}
		while(ticks_to_dist(driveBase.left.last_ticks - driveBase.right.last_ticks) > kCircumference/4) {
			driveBase = turn_left(driveBase);
		}
		while(ticks_to_dist(driveBase.right.last_ticks - driveBase.left.last_ticks) > kCircumference/4) {
			driveBase = turn_right(driveBase);
		}
		delay(kDecelerationDelay);
		driveBase = realign(driveBase, 0);

		for(int i = 0; i < 3; ++i) {
			driveBase = move_to_square(driveBase, REAR, 3, true);
		}
		driveBase = move_to_square(driveBase, REAR, 2, true);
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