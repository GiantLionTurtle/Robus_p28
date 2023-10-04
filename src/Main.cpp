
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
	// MOTOR_SetSpeed(RIGHT, 0.5);

	// driveBase = p28::turn_left(driveBase);
	// delay(500);
	// driveBase = p28::turn_right(driveBase);
	// delay(500);
	driveBase = p28::forward_dist(driveBase, 2, 0.2);

	// driveBase = p28::solve(driveBase);
}

void loop() 
{
	Serial.println("out");
	delay(10);
}