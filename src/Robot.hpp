
#ifndef P28_ROBOTSTATE_HPP_
#define P28_ROBOTSTATE_HPP_

#include "Utils/Vec2.hpp"
#include "Subsystems/Drivebase.hpp"
#include "SensorsState.hpp"
#include "Iteration_time.hpp"
#include "Subsystems/Bin.hpp"
#include "Subsystems/Conveyor.hpp"

namespace p28 {

struct DumpObjective {
	enum Steps {
		Start,
		GetToLine, 		// Paths toward a line on the ground
		AlignToLine,	// Turn ccw to be aligned to line
		GetToDump,		// Use path follower to get to zone
		DoDump,			// Release blocs
		Done
	};
	int step;
};

struct Robot {
	// Subsystems
	Bin bin;
	Drivebase drvb;
	Conveyor cnvr;


	mt::Vec2 headingMemory { 0.0 };
	mt::Vec2 posMemory { 0.0 };

	int nFrames_noLegos { 0 }; // Dont switch back to path for a few frames
	time_t trapReleaseTimer { 0 };
	bool waitInstruct {false};


	int targetColor { kRed };
	int drop_zone;	// Drop zone is defined by targetColor

	DumpObjective dumpObjective { DumpObjective::Done };

	void init();
	void start_calibration();
	void start_search();

	// Compute the next robot state from delta of the sensors and the game state
	void update(SensorState prevSensState, SensorState currSensState, Iteration_time it_time);
	HardwareState generate_hardwareState(Iteration_time it_time);

	void gameLogic(SensorState const& currSensState, SensorState const& prevSensState, Iteration_time it_time);

	void set_target_color(int controller_color);
	void huntLogic(SensorState sensState, Iteration_time it_time);

	void dumpObjective_helper(SensorState const& currSensState, Iteration_time it_time);
};



} // !p28

#endif