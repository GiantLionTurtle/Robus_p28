
#include "GameState.hpp"
#include "Constants.hpp"
#include "Field.hpp"
#include "sensors.hpp"
#include "CompileFlags.hpp"


namespace p28 {

Pair<int, int> compute_zoneLane(SensorState  const& prevSensState, SensorState const&  currSensState, GameState const&  gmState, DrivebaseState drvbState);
Objective compute_knockCup_state(GameState const& gmState, SensorState const& sensState, time_t time_ms);
Objective compute_one_turn_state(GameState const& gmState, GameState const& previousGmState, time_t time_ms);
Objective compute_tests_state(GameState const& gmState, time_t time_ms);
int comp_lane(COLOR color);

GameState GameState::initial(SensorState sensState)
{
	GameState initial_gameState;
	initial_gameState.over = false;
	initial_gameState.lane = comp_lane(sensState.colorDetector);
	initial_gameState.target_lane = initial_gameState.lane;
	initial_gameState.missions.test.donneness = Objective::Todo;
	
	return initial_gameState;
}

#ifndef RACE_MODE
GameState GameState::generate_next(SensorState prevSensState, SensorState currSensState, DrivebaseState drvbState,Iteration_time it_time) const
{
	GameState newGmState = *this;

	// Figure out where we are in game zones
	tie(newGmState.zone, newGmState.lane) = compute_zoneLane(prevSensState, currSensState, newGmState, drvbState);

	if(newGmState.zone != zone) {
		Serial.print("zone: ");
		Serial.println(newGmState.zone);
	}
	// Figure out what to do now
	newGmState.missions.one_turn = compute_one_turn_state(newGmState, *this, it_time.time_ms);
	newGmState.missions.knock_cup = compute_knockCup_state(newGmState, currSensState, it_time.time_ms);
	newGmState.missions.test = compute_tests_state(newGmState, it_time.time_ms);

	// ping pong
	// shortcut
	return newGmState;
}
#else
GameState GameState::generate_next(SensorState prevSensState, SensorState currSensState, DrivebaseState drvbState,Iteration_time it_time) const
{
	
}
#endif


// Try to deduce de lane based on the color sensor
int comp_lane(COLOR color)
{
	switch(color) {
	case COLOR::BLUE: 	return 0;
	case COLOR::GREEN: 	return 1;
	case COLOR::YELLOW: return 2;
	case COLOR::RED: 	return 3;
	default: 			return -1;
	}
	return -1;
}

Pair<int, int> compute_zoneLane(SensorState const& prevSensState, SensorState const& currSensState, GameState const& gmState, DrivebaseState drvbState)
{
	int zone = gmState.zone;
	int lane = gmState.lane;

// Figure out zone	
#ifdef LEFT_BUMPER_FOR_ZONE_INCREMENT
	if(prevSensState.bumpersState.left == 0 && currSensState.bumpersState.left == 1)
		zone++;
#else
	if(currSensState.colorDetector == COLOR::BLACK){
		zone = 2;
	} else if(currSensState.colorDetector == COLOR::WHITE
			&& prevSensState.colorDetector != COLOR::WHITE) {
		zone = 6;
	} else {
		if(prevSensState.colorDetector == COLOR::WHITE)
			zone = 9;
	}
	for(int i = 0; i < Field::n_zones; ++i) {
		if(Field::zones_boxes[i].point_inside(drvbState.pos)) {
			zone = i;
			break;
		}
	}
#endif

// Figure out lane
	if(currSensState.colorDetector != COLOR::BLACK && currSensState.colorDetector != COLOR::WHITE) {
		lane = comp_lane(currSensState.colorDetector);
	}



	return { zone, lane };
}

Objective compute_knockCup_state(GameState const& gmState, SensorState const& sensState, time_t time_ms)
{
	bool start_cond = (gmState.zone == 4 || gmState.zone == 5) && // right zones
						sensState.backIR_dist <= kCupDetectDist; // Sensor detects
	bool end_cond = (gmState.zone > 5) || time_ms-gmState.missions.knock_cup.start_ms > kKnockCupDelay;

	return gmState.missions.knock_cup.advance(start_cond, end_cond, time_ms);
}
Objective compute_one_turn_state(GameState const& gmState, GameState const& previousGmState, time_t time_ms)
{
	bool start_cond = true;
	bool end_cond = gmState.zone == 9 && previousGmState.zone == 8;

	return gmState.missions.one_turn.advance(start_cond, end_cond, time_ms);
}
Objective compute_one_shortCut_state(GameState const& gmState,GameState const& previousGmState, time_t time_ms)
{
	bool start_cond = gmState.zone == 5 && gmState.missions.one_turn.done();
	bool end_cond = gmState.zone == 9 && previousGmState.zone == Field::kshortcutZone;	
	
	return gmState.missions.one_shortcut_turn.advance(start_cond, end_cond, time_ms);
}
Objective compute_tests_state(GameState const& gmState, time_t time_ms)
{
#ifdef ENABLE_TEST_OBJECTIVE
	bool start_cond = true;
#else 
	bool start_cond = false;
#endif
	bool end_cond = false;
	
	return gmState.missions.test.advance(start_cond, end_cond, time_ms);
}
} // !p28