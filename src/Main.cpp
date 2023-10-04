
#include <Arduino.h>
#include "LibRobus.h"
#include "Constants.hpp"
#include "Drivebase.hpp"
#include "Solver.hpp"

#define ROBOT_A

p28::Drivebase driveBase;

void setup()
{
	BoardInit();
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
	pinMode(14, INPUT);
    pinMode(15, INPUT);
	bool detect = false;

	//MOTOR_SetSpeed(RIGHT, 0.5);

	// driveBase = p28::turn_left(driveBase);
	// delay(500);
	// driveBase = p28::turn_right(driveBase);
	// delay(500);
	driveBase = p28::forward_dist(driveBase, 2, 0.2);

	// driveBase = p28::solve(driveBase);
	// // driveBase = p28::forward_dist(driveBase, 0.5, 0.2);

	// driveBase = p28::solve(driveBase);
	driveBase= p28::move_to_square(driveBase, REAR, 1);
}

void loop() 
{
	//Serial.println("out");
	delay(10);
}