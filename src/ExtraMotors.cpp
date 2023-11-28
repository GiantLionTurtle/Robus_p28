
#include <LibRobus.h>
#include "ExtraMotors.hpp"
#include <Stepper.h>

namespace p28 {

void ExtraMotors::init() 
{
	bin_red.attach(38);
	bin_green.attach(40);
	bin_blue.attach(42);
	trap.attach(44);
	conveyor.setSpeed(10);
}

} // !p28

