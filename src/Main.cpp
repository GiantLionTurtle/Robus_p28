
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

	delay(1000);
	driveBase.left.ID = LEFT;
	driveBase.right.ID = RIGHT;

#ifdef ROBOT_A
	driveBase.left.pid = { 2.8, 53.4, 0.055 };
	driveBase.right.pid = { 2.8, 53.4, 0.055 };
#else
	driveBase.left.pid = { 2.8, 53.4, 0.055 };
	driveBase.right.pid = { 2.8, 53.4, 0.055 };
#endif

	//MOTOR_SetSpeed(RIGHT, 0.5);

	// driveBase = turn_left(driveBase);
	// delay(500);
	// driveBase = turn_right(driveBase);
	// delay(500);
	//driveBase = forward_dist(driveBase, 2, 0.2);

	// driveBase = solve(driveBase);
	// // driveBase = forward_dist(driveBase, 0.5, 0.2);

	// driveBase = solve(driveBase);
	// driveBase= move_to_square(driveBase, REAR, 1);
	// driveBase = move_to_square_or_detect(driveBase, RIGHT, detect);
	//driveBase = solve2(driveBase);

	// Serial.print("Legal: ");
	// Serial.println(is_move_legal(1, 0, RIGHT));
	// driveBase= move_to_square(driveBase, REAR, 1);
}

void loop() 
{
	bool start = whistle_detection();
	if(start)
	{
		driveBase = solve2(driveBase);
	}
	delay(10);
}