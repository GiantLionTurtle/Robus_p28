
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

	// MOTOR_SetSpeed(RIGHT, 0.5);

	mainBase = p28::turn_left(mainBase);
	delay(500);
	mainBase = p28::turn_right(mainBase);
	delay(500);
	mainBase = p28::turn_left(mainBase);
	mainBase = p28::turn_left(mainBase);
	mainBase = p28::turn_left(mainBase);
	mainBase = p28::turn_left(mainBase);

}

void loop() 
{
	Serial.println("out");
	delay(10);
}