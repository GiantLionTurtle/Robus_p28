#include "HardwareState.hpp"
#include <LibRobus.h>
#include "Iteration_time.hpp"
#include "Constants.hpp"


namespace p28 {

struct Conveyor{
	int sequenceIndex = 4;
	unsigned long startStepTime = 0.0;
	
	void init();
	bool over() const;
	bool just_dropped() const;
	bool in_clawMove() const;
	void start_sequence(Iteration_time it_time);
	void update(Iteration_time it_time);
	HardwareState aggregate(HardwareState hrdwState);
};

}