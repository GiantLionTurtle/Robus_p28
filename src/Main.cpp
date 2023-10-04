
#include <Arduino.h>
#include "LibRobus.h"
#include "Constants.hpp"
#include "Drivebase.hpp"
#include "Solver.hpp"
#include "WhistleDetector.hpp"

#define ROBOT_A

p28::Drivebase driveBase;
bool detect = false;

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
	pinMode(18, INPUT);

	// driveBase = p28::forward_until_detect(driveBase, 5, 0.2, detect);
	// delay(100);
	// driveBase = p28::turn_right(driveBase);
	// delay(100);
	// driveBase = p28::turn_right(driveBase);
	// delay(100);
	// driveBase = p28::forward_dist(driveBase, 0.5, 0.2);
}

void loop() 
{
	if(whistle_detection())
	{
		detect = false;
		driveBase = p28::forward_until_detect(driveBase, 5, 0.2, detect);
		delay(100);
		driveBase = p28::turn_right(driveBase);
		delay(100);
		driveBase = p28::turn_right(driveBase);
		delay(100);
	}
	delay(10);
}