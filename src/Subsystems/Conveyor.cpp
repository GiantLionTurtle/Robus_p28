
#include "Conveyor.hpp"
#include "Constants.hpp"
#include <SensorsState.hpp>
#include "Iteration_time.hpp"



namespace p28 {

struct Step{
	float clawServo;
	float armServo;
	int conveyorSteps;
	unsigned long stepTime;
};
const int Nsteps = 7;
Step const sequence[Nsteps] = {
	Step { .clawServo = kClaw_openAngle, .armServo = kArm_restAngle, .conveyorSteps = 0, .stepTime = 60},
	Step { .clawServo = kClaw_openAngle, .armServo = kArm_downAngle, .conveyorSteps = 0, .stepTime = 60},
	Step { .clawServo = kClaw_closeAngle, .armServo = kArm_downAngle, .conveyorSteps = 0, .stepTime = 150},
	Step { .clawServo = kClaw_closeAngle, .armServo = kArm_upAngle, .conveyorSteps = 0, .stepTime = 300},
	Step { .clawServo = kClaw_openAngle, .armServo = kArm_upAngle, .conveyorSteps = 0, .stepTime = 50},
	Step { .clawServo = kClaw_openAngle, .armServo = kArm_upAngle, .conveyorSteps = kConveyor_stepsUntilUp, .stepTime = 3000}, // #define delay 
	Step { .clawServo = kClaw_openAngle, .armServo = kArm_upAngle, .conveyorSteps = -kConveyor_stepsUntilUp, .stepTime = 3000} // #define delay
};


void Conveyor::start_sequence(Iteration_time it_time)
{
	startStepTime = it_time.time_ms;
	sequenceIndex = 0; // First index
}
void Conveyor::start_squenceIfDown(Iteration_time it_time)
{
	if(sequenceIndex < Nsteps)
		return;
	start_sequence(it_time);
}
void Conveyor::update(Iteration_time it_time)
{
	if (sequenceIndex < Nsteps){
		if ((it_time.time_ms - startStepTime) >= sequence[sequenceIndex].stepTime) {
			sequenceIndex++;
			startStepTime = it_time.time_ms;
		}
	}
	
}
HardwareState Conveyor::aggregate(HardwareState hrdwState)
{
	hrdwState.armAngle = sequence[sequenceIndex%Nsteps].armServo;
	hrdwState.clawAngle = sequence[sequenceIndex%Nsteps].clawServo;
	hrdwState.conveyorSteps = sequence[sequenceIndex%Nsteps].conveyorSteps;
	return hrdwState;
}

}