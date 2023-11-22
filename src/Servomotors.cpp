#include <LibRobus.h>
#include "Servomotors.hpp"

void ExtraMotor::init() 
{
bin_red.attach(38);               // 
bin_green.attach(40);
bin_blue.attach(42);
trap.attach(44);
}

