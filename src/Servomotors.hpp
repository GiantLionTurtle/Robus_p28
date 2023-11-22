
#ifndef P28_SERVOMOTORS_HPP
#define P28_SERVOMOTORS_HPP
#include <LibRobus.h>

struct ExtraMotor {
MegaServo bin_red;     // bin1 is the servo for red
MegaServo bin_green;     // bin2 is the servo for green
MegaServo bin_blue;     // bin3 is the servo for blue
MegaServo trap;     // servo for the trap

void init();

};


#endif


