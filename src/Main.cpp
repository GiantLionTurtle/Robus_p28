
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

	// driveBase= move_to_square(driveBase, FRONT, 1);
	// bool detect;
	// driveBase = move_to_square_or_detect(driveBase, FRONT, 1, detect);

	Serial.print("Legal: ");
	Serial.println(is_move_legal(0, 1, RIGHT));
	// driveBase= move_to_square(driveBase, REAR, 1);

	// for(int i = 0; i < 4; ++i) {
	// 	// driveBase = turn_right(driveBase, 1);
	// }
}

void loop() 
{
	if(ROBUS_IsBumper(3)) {
		driveBase = solve3(driveBase);
	}
	// driveBase = realign(driveBase);
	// driveBase = turn_left(driveBase);
	// bool start = whistle_detection();
	// if(start)
	// {
	// 	driveBase = solve2(driveBase);
	// }
	// delay(10);
}