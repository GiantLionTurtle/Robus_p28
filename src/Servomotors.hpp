
#ifndef P28_SERVOMOTORS_HPP
#define P28_SERVOMOTORS_HPP
#include <LibRobus.h>
#include <Stepper.h>
#include "Constants.hpp"


struct ExtraMotor {
	MegaServo bin_red;     // bin1 is the servo for red
	MegaServo bin_green;     // bin2 is the servo for green
	MegaServo bin_blue;     // bin3 is the servo for blue
	MegaServo trap;     // servo for the trap
	Stepper conveyor = Stepper(p28::kConveyor_stepsPerRevolution, 39, 43, 41, 45);

	void init();
};


#endif


