
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
const int Nsteps = 8;
const int CimbStepInd = 5; // Step at which the stepper starts, keep this linked
Step const sequence[Nsteps] = {
	Step { .clawServo = kClaw_openAngle, .armServo = kArm_upAngle, .conveyorSteps = 0, .stepTime = 200},
	Step { .clawServo = kClaw_openAngle, .armServo = kArm_downAngle, .conveyorSteps = 0, .stepTime = 1000},
	Step { .clawServo = kClaw_closeAngle, .armServo = kArm_downAngle, .conveyorSteps = 0, .stepTime = 600},
	Step { .clawServo = kClaw_closeAngle, .armServo = kArm_upAngle, .conveyorSteps = 0, .stepTime = 600},
	Step { .clawServo = kClaw_openAngle, .armServo = kArm_upAngle, .conveyorSteps = 0, .stepTime = 50},
	Step { .clawServo = kClaw_openAngle, .armServo = kArm_upAngle, .conveyorSteps = 0, .stepTime = 200},
	Step { .clawServo = kClaw_openAngle, .armServo = kArm_upAngle, .conveyorSteps = kConveyor_stepsUntilUp, .stepTime = 16000}, // #define delay 
	Step { .clawServo = kClaw_openAngle, .armServo = kArm_upAngle, .conveyorSteps = 0, .stepTime = 16000} // #define delay
};

void Conveyor::init()
{
	sequenceIndex = Nsteps;
}
void Conveyor::start_sequence(Iteration_time it_time)
{
	startStepTime = it_time.time_ms;
	sequenceIndex = 0; // First index
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
bool Conveyor::over() const
{
	if(sequenceIndex < Nsteps)
		return false;
	return true;
}
bool Conveyor::just_dropped() const
{
	// Serial.print("Seq ind ");
	// Serial.println(sequenceIndex);
	if(sequenceIndex == CimbStepInd)
		return true;
	return false;
}

}