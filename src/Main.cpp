
#include <Arduino.h>
#include "LibRobus.h"
#include "Constants.hpp"
#include "Drivebase.hpp"

p28::Motor motorLeft;
p28::Motor motorRight;



void setup() 
{
	BoardInit();
	motorLeft.ID = LEFT;
	motorLeft.pid = { 1.4, 35.55555, 0.0333333 };

	motorRight.ID = RIGHT;
	motorRight.pid = { 1.4, 35.55555, 0.0333333 };
}

void loop() 
{
	long int millis_time = millis();
	motorLeft = p28::update_motor_at_speed(motorLeft, 0.5, millis_time);
	motorRight = p28::update_motor_at_speed(motorLeft, 0.5, millis_time);

	delay(10);
}