
#include <Arduino.h>
#include "LibRobus.h"
#include "Constants.hpp"
#include "Drivebase.hpp"

p28::Motor motorLeft;
p28::Motor motorRight;

float ticks_to_dist(uint32_t ticks){
	return static_cast<float>(ticks) / 3200.0 * TWO_PI * p28::kWheelRadius;
}

void setup(){
	BoardInit();
	motorLeft.pid = { 1.4, 35.55555, 0.0333333 };
	motorRight.pid = { 1.4, 35.55555, 0.0333333 };
}

void loop(){
	static long int TempsAvant =0;
	long int millis_time = millis();
	motorLeft = p28::update_motor_at_speed(motorLeft, 0.5, millis_time);
	motorRight = p28::update_motor_at_speed(motorRight, 0.5, millis_time);
    
}
