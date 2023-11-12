
#include "conveyor.hpp"
#include "Constants.hpp"
#include <SensorsState.hpp>

namespace p28 {
    HardwareState update_conveyorSequence(SensorState sensorState,Conveyor previousState){
        Conveyor state;//previous conveyor state
        HardwareState hrdwState;
        
        if (sensorState.pixy_legoDist==true){
            state.conveyorSequence = 1;
            hrdwState.clawAngle = kClaw_closeAngle;
        }
        else if (previousState.conveyorSequence == 1 && previousState.sequenceTime >=1000 ){
            state.conveyorSequence = 2;
            hrdwState.armAngle = kArm_closeAngle;
        }
        else if (previousState.conveyorSequence == 2 && previousState.sequenceTime >=2000 ){
            state.conveyorSequence = 3;
            hrdwState.clawAngle = kClaw_openAngle;
        }    
        else if (previousState.conveyorSequence == 3 && previousState.sequenceTime >=1000){
            state.conveyorSequence = 4;
            hrdwState.armAngle = kArm_openAngle;
        }
        else if (previousState.conveyorSequence == 4){
            state.conveyorSequence = 0;
        }
        return hrdwState;
    }

}