#include "HardwareState.hpp"
#include <LibRobus.h>
#include "Iteration_time.hpp"


namespace p28 {

struct Conveyor{
	int sequenceIndex = 4;
	unsigned long startStepTime = 0.0;
	
	void start_sequence(Iteration_time it_time);
	void start_squenceIfDown(Iteration_time it_time);
	void update(Iteration_time it_time);
	HardwareState aggregate(HardwareState hrdwState);
};

}