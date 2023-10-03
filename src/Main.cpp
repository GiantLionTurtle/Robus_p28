
#include <Arduino.h>
#include "LibRobus.h"
#include "Constants.hpp"
#include "Drivebase.hpp"
#include "Solver.hpp"

p28::Drivebase driveBase;

void setup() 
{
	BoardInit();
	delay(1000);
	Serial.println("Begin!");

	driveBase.left.ID = LEFT;
	driveBase.left.pid = { 1.4, 35.55555, 0.0333333 };

	driveBase.right.ID = RIGHT;
	driveBase.right.pid = { 1.4, 35.55555, 0.0333333 };

	// MOTOR_SetSpeed(RIGHT, 0.5);

	driveBase = p28::turn_left(driveBase);
	delay(500);
	driveBase = p28::turn_right(driveBase);
	delay(500);
	// driveBase = p28::forward_dist(driveBase, 0.5, 0.2);

	driveBase = p28::solve(driveBase);
}

void loop() 
{
	Serial.println("out");
	delay(10);
}