#include "HardwareState.hpp"
#include "sensors.hpp"
#include <LibRobus.h>


namespace p28 {

    struct Conveyor{
        int conveyorSequence;
        float sequenceTime = millis();
    };

}