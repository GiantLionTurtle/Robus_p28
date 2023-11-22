
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
const int Nsteps = 6;
Step const sequence[Nsteps] = {
	Step { .clawServo = kClaw_openAngle, .armServo = kArm_openAngle, .conveyorSteps = 0, .stepTime = 0},
	Step { .clawServo = kClaw_closeAngle, .armServo = kArm_openAngle, .conveyorSteps = 0, .stepTime = 150},
	Step { .clawServo = kClaw_closeAngle, .armServo = kArm_closeAngle, .conveyorSteps = 0, .stepTime = 300},
	Step { .clawServo = kClaw_openAngle, .armServo = kArm_closeAngle, .conveyorSteps = 0, .stepTime = 50},
	Step { .clawServo = kClaw_openAngle, .armServo = kArm_closeAngle, .conveyorSteps = kConveyor_stepsUntilUp, .stepTime = 50}, // #define delay 
	Step { .clawServo = kClaw_openAngle, .armServo = kArm_closeAngle, .conveyorSteps = -kConveyor_stepsUntilUp, .stepTime = 50} // #define delay
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