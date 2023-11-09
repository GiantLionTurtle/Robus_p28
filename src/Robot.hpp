
#ifndef P28_ROBOTSTATE_HPP_
#define P28_ROBOTSTATE_HPP_

#include "Utils/Vec2.hpp"
#include "Drivebase.hpp"
#include "sensors.hpp"
#include "GameState.hpp"
#include "Iteration_time.hpp"

/*
	How the Robot should work

	It contains a representation of all subsistems.
	
	1. It generates a new copy of itself using sensor data, gamestate data and time
*/

namespace p28 {

// Essentialy proprioception for the robot
struct Robot {
	bool openArm { false };
	bool releaseCup { false };

	Drivebase drvb;

	static Robot initial(GameState gmState);

	// Compute the next robot state from delta of the sensors and the game state
	void generate_next(	SensorState prevSensState, SensorState currSensState, 
								GameState prevGmState, GameState gmState, Iteration_time it_time);

	void test_helper(GameState gmState, Iteration_time it_time);
	void knockCup_helper(GameState gmState, Iteration_time it_time);
	void trapBall_helper(GameState gmState, Iteration_time it_time);
	void oneTurn_helper(GameState gmState, Iteration_time it_time);
	void shortCut_helper(GameState gmState, Iteration_time it_time);
	void followLine();
	void adjustDrivebase(SensorState const& currSensState, SensorState const& prevSensState,
								GameState const& prevGmState, GameState const& gmState);
};

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

}; // !p28

}

#endif