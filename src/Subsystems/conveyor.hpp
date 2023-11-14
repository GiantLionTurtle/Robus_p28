#include "HardwareState.hpp"
#include <LibRobus.h>


namespace p28 {

    struct Conveyor{
        int conveyorSequence = -1;
        float sequenceTime = millis();
    };

}