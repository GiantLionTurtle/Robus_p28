#include "HardwareState.hpp"
#include <LibRobus.h>
#include "Iteration_time.hpp"


namespace p28 {

    struct Conveyor{
        int conveyorSequence = 4;
        unsigned long startStepTime = 0.0;
        HardwareState update_conveyorSequence(Conveyor conveyorState, HardwareState hrdwState,Iteration_time it_time);
        Conveyor Start_ConveyorTime(Iteration_time it_time, Conveyor conveyorState);
    };

}