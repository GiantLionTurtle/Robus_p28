
#ifndef P28_ROBOTSTATE_HPP_
#define P28_ROBOTSTATE_HPP_

#include "Utils/Vec2.hpp"
#include "Subsystems/Drivebase.hpp"
#include "SensorsState.hpp"
#include "Iteration_time.hpp"
#include "Subsystems/Bin.hpp"
#include "Subsystems/Conveyor.hpp"

/*
	How the Robot should work

	It contains a representation of all subsistems.
	
	1. It generates a new copy of itself using sensor data, gamestate data and time
*/

namespace p28 {

struct Objective {
	enum Doneness : char { Todo, Start, UnderWay, Done };

	Doneness donneness;
	time_t start_ms;

	bool todo() const { return donneness == Doneness::Todo; }
	bool start() const { return donneness == Doneness::Start; }
	bool underway() const { return donneness == Doneness::UnderWay; }
	bool done() const { return donneness == Doneness::Done; }

	Objective advance(bool start_cond, bool end_cond, time_t time_ms) const
	{
		if(todo() && start_cond) {
			return Objective { .donneness=Doneness::Start, .start_ms=time_ms };
		}
		if(start()) {
			return Objective { .donneness=Doneness::UnderWay, .start_ms=start_ms };
		}
		if(underway() && end_cond) {
			return Objective { .donneness=Doneness::Done, .start_ms=start_ms };
		}
		return *this;
	}

};

struct DumpObjective {
	enum Steps {
		Start,
		GetToLine,
		GetToDump,
		DoDump,
		Done
	};
	int step;
};

// Essentialy proprioception for the robot
struct Robot {
	
	Bin bin;
	Drivebase drvb;
	Conveyor cnvr;
	mt::Vec2 headingMemory { 0.0 };
	mt::Vec2 posMemory { 0.0 };

	int nFrames_noLegos { 0 };
	// Do not reset pos memory & heading memory
	// if the path is not at least at this index
	// (it is still trying to get back on path)
	int backToPath_index { kMaxCheckPointForPath }; 
	int drop_zone;
	time_t trapReleaseTimer { 0 };

	int nBlocksInCycle { 0 };

	int targetColor { kRed };
	DumpObjective dumpObjective { DumpObjective::Done };

	void init();
	void start_calibration();

	// Compute the next robot state from delta of the sensors and the game state
	void update(SensorState prevSensState, SensorState currSensState, Iteration_time it_time);
	HardwareState generate_hardwareState(Iteration_time it_time);

	void gameLogic(SensorState const& currSensState, SensorState const& prevSensState, Iteration_time it_time);

	void set_target_color(int controller_color);
	void huntLogic(SensorState sensState, Iteration_time it_time);

	void dumpObjective_helper(Iteration_time it_time);
};



} // !p28

#endif