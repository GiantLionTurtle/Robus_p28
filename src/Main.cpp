
#include <Arduino.h>
#include "LibRobus.h"
#include "Constants.hpp"
#include "Drivebase.hpp"

<<<<<<<<< Temporary merge branch 1
void PrintEmile();

void setup() 
{

=========
p28::Motor motorLeft;
p28::Motor motorRight;

float ticks_to_dist(uint32_t ticks)
{
	return static_cast<float>(ticks) / 3200.0 * TWO_PI * p28::kWheelRadius;
}

void setup() 
{
	BoardInit();
	motorLeft.pid = { 1.4, 35.55555, 0.0333333 };
	motorRight.pid = { 1.4, 35.55555, 0.0333333 };
>>>>>>>>> Temporary merge branch 2
}

void loop() 
{
<<<<<<<<< Temporary merge branch 1
    
}

void PrintEmile(){
    printf("Emile Raymond test 3"); 
=========
	long int millis_time = millis();
	motorLeft = p28::update_motor_at_speed(motorLeft, 0.5, millis_time);
	motorRight = p28::update_motor_at_speed(motorLeft, 0.5, millis_time);

	delay(10);
>>>>>>>>> Temporary merge branch 2
}