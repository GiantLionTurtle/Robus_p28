
#include <Arduino.h>
#include "LibRobus.h"
#include "Constants.hpp"
#include "Drivebase.hpp"
#include "Solver.hpp"
#include "WhistleDetector.hpp"
#include "ProximityDetector.hpp"

Drivebase driveBase;
bool start = false;

void setup()
{
	BoardInit();
	Serial.println("Begin!");

	init_detector();
	init_whistle();
	init_legalityMatrix();
	buzzerFin();

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
	bool fail = false;
	driveBase = solve2(driveBase, fail);

	// Serial.print("Legal: ");
	// Serial.println(is_move_legal(1, 0, RIGHT));
	// driveBase= move_to_square(driveBase, REAR, 1);
}

void loop() 
{
	// if(start)
	// {
	// 	bool detect = false;
	// 	float traveled_dist = 0;
	// 	driveBase = forward_until_detect(driveBase, 5, 0.2, traveled_dist, detect);
	// 	delay(100);
	// 	driveBase = turn_right(driveBase);
	// 	delay(100);
	// 	driveBase = turn_right(driveBase);
	// 	delay(100);
	// }
	// if(whistle_detection())
	// {
	// 	start = true;
	// }
	// Serial.println("Whistle: ");
	// Serial.println(whistle_detection());
	// delay(10);
}