
#include <Arduino.h>
#include "LibRobus.h"
#include "Constants.hpp"
#include "Drivebase.hpp"
#include "Solver.hpp"
#include "ProximityDetector.hpp"

p28::Drivebase driveBase;

void setup()
{
	BoardInit();
	p28::init_detector();
	p28::init_legalityMatrix();
	delay(1000);
	Serial.println("Begin!");

	driveBase.left.ID = LEFT;
	driveBase.right.ID = RIGHT;

#ifdef ROBOT_A
	driveBase.left.pid = { 2.8, 53.4, 0.055 };
	driveBase.right.pid = { 2.8, 53.4, 0.055 };
#else
	driveBase.left.pid = { 2.8, 53.4, 0.055 };
	driveBase.right.pid = { 2.8, 53.4, 0.055 };
#endif

	bool detect = false;

	//MOTOR_SetSpeed(RIGHT, 0.5);

	// driveBase = p28::turn_left(driveBase);
	// delay(500);
	// driveBase = p28::turn_right(driveBase);
	// delay(500);
	// driveBase = p28::forward_dist(driveBase, 2, 0.2);

	// driveBase = p28::solve(driveBase);
	// // driveBase = p28::forward_dist(driveBase, 0.5, 0.2);

	// driveBase = p28::solve(driveBase);
	// driveBase= p28::move_to_square(driveBase, REAR, 1);
	// driveBase = p28::move_to_square_or_detect(driveBase, RIGHT, detect);
	driveBase = p28::solve2(driveBase);
}

void loop() 
{
	//Serial.println("out");
	delay(10);
}