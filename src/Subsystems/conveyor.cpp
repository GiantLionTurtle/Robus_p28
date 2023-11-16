
#include "conveyor.hpp"
#include "Constants.hpp"
#include <SensorsState.hpp>
#include "Iteration_time.hpp"





namespace p28 {

    struct Step{
        float clawServo;
        float armServo;
        long stepTime;


    };
    const int Nsteps = 4;
    Step sequence[Nsteps] = {
        Step { .clawServo = kClaw_openAngle, .armServo = kArm_openAngle, .stepTime =2000},
        Step { .clawServo = kClaw_closeAngle, .armServo = kArm_openAngle, .stepTime = 2000},
        Step { .clawServo = kClaw_closeAngle, .armServo = kArm_closeAngle, .stepTime = 2000},
        Step { .clawServo = kClaw_openAngle, .armServo = kArm_closeAngle, .stepTime =2000}
        
    };
    Conveyor Start_ConveyorTime(Iteration_time it_time, Conveyor conveyorState){
        conveyorState.startStepTime = it_time.time_ms;
        conveyorState.conveyorSequence = 0;
        return conveyorState;
    }


    HardwareState update_conveyorSequence(Conveyor conveyorState, Step sequence[], HardwareState hrdwState,Iteration_time it_time ){
        
        int stepIndex = conveyorState.conveyorSequence;
        if ( stepIndex < 4){
            hrdwState.armAngle = sequence[stepIndex].armServo;
            hrdwState.clawAngle = sequence[stepIndex].clawServo;
            if (it_time.time_ms - conveyorState.startStepTime >= sequence[stepIndex].stepTime){
                stepIndex++;
                conveyorState.startStepTime = it_time.time_ms;
            }
        }
        else if (stepIndex == 4){
            hrdwState.armAngle = sequence[0].armServo;
            hrdwState.clawAngle = sequence[0].clawServo;
        }
        
        return hrdwState;
    }
}