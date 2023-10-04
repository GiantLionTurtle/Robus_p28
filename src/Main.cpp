
#include <Arduino.h>
#include "LibRobus.h"
#include "Constants.hpp"
#include "Drivebase.hpp"

p28::Drivebase mainBase;

void setup() 
{
	BoardInit();
	delay(1000);
	Serial.println("Begin!");

	mainBase.left.ID = LEFT;
	mainBase.left.pid = { 1.4, 35.55555, 0.0333333 };

	mainBase.right.ID = RIGHT;
	mainBase.right.pid = { 1.4, 35.55555, 0.0333333 };

	pinMode(14, INPUT);
    pinMode(15, INPUT);
	bool detect = false;

	//MOTOR_SetSpeed(RIGHT, 0.5);

	// /*mainBase = p28::turn_left(mainBase);
	// delay(500);
	// mainBase = p28::turn_right(mainBase);
	// delay(500);
	mainBase = p28::forward_until_detect(mainBase, 5, 0.2, detect);
	delay(100);
	mainBase = p28::turn_right(mainBase);
	delay(100);
	mainBase = p28::turn_right(mainBase);
	delay(100);
	mainBase = p28:: forward_dist(mainBase, 0.5, 0.2);
}

void loop() 
{
	Serial.println("Out of setup");
	delay(10);
}