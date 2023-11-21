
#include "Conveyor.hpp"
#include "Constants.hpp"
#include <SensorsState.hpp>
#include "Iteration_time.hpp"


namespace p28 {

struct Step{
	float clawServo;
	float armServo;
	unsigned long stepTime;
};
const int Nsteps = 4;
Step const sequence[Nsteps] = {
	Step { .clawServo = kClaw_openAngle, .armServo = kArm_openAngle, .stepTime = 0},
	Step { .clawServo = kClaw_closeAngle, .armServo = kArm_openAngle, .stepTime = 150},
	Step { .clawServo = kClaw_closeAngle, .armServo = kArm_closeAngle, .stepTime = 300},
	Step { .clawServo = kClaw_openAngle, .armServo = kArm_closeAngle, .stepTime = 50}
	
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
	return hrdwState;
}

}