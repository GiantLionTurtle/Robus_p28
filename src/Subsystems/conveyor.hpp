#include "HardwareState.hpp"
#include <LibRobus.h>


namespace p28 {

    struct Conveyor{
        int conveyorSequence = 4;
        unsigned long startStepTime = 0.0;
    };

}