#include "HardwareState.hpp"
#include <LibRobus.h>


namespace p28 {

    struct Conveyor{
        int conveyorSequence;
        float sequenceTime = millis();
    };

}