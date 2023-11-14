
#include "conveyor.hpp"
#include "Constants.hpp"
#include <SensorsState.hpp>
#include "Iteration_time.hpp"





namespace p28 {

    struct Step{
        float clawServo;
        float armServo;
        long delayAction;


    };
    const int Nsteps = 4;
    Step sequence[Nsteps] = {
        Step { .clawServo = kClaw_openAngle, .armServo = kArm_openAngle, .delayAction =2000},
        Step { .clawServo = kClaw_closeAngle, .armServo = kArm_openAngle, .delayAction = 2000},
        Step { .clawServo = kClaw_closeAngle, .armServo = kArm_closeAngle, .delayAction = 2000},
        Step { .clawServo = kClaw_openAngle, .armServo = kArm_closeAngle, .delayAction =2000}
        
    };



    HardwareState update_conveyorSequence(Conveyor conveyorState, Step sequence, HardwareState hrdwState,Iteration_time it_time ){
        
        int stepIndex = conveyorState.conveyorSequence;
        unsigned long sequenceTime = conveyorState.sequenceTime;
        
        
        if (stepIndex = -1 & stepIndex < 4){
            if (sequence[stepIndex])
        }
        
        return hrdwState;
     }
}