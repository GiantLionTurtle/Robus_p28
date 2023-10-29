
#include "GameState.hpp"
#include "Constants.hpp"
#include "Field.hpp"


namespace p28 {

Pair<int, int> compute_zoneLane(SensorState  const& prevSensState, SensorState const&  currSensState, GameState const&  gmState, DrivebaseState drvbState);
Objective compute_knockCup_state(GameState const& gmState);
Objective compute_one_cw_turn_state(GameState const& gmState,GameState const& previousGmState);
int comp_lane(COLOR color);

GameState GameState::initial(SensorState sensState)
{
	GameState initial_gameState;
	initial_gameState.lane = comp_lane(sensState.colorDetector);
	initial_gameState.target_lane = initial_gameState.lane;
}
GameState GameState::generate_next(SensorState prevSensState, SensorState currSensState, DrivebaseState drvbState,Iteration_time it_time) const
{
	GameState newGmState = *this;

	// Figure out where we are in game zones
	tie(newGmState.zone, newGmState.lane) = compute_zoneLane(prevSensState, currSensState, newGmState, drvbState);


	// Figure out what to do now
	newGmState.missionState.knock_cup = compute_knockCup_state(newGmState);
	// ping pong
	// shortcut
	return newGmState;

}

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
	if(currSensState.colorDetector == COLOR::BLACK){
		zone = 2;
	} else if(currSensState.colorDetector == COLOR::WHITE
			&& prevSensState.colorDetector != COLOR::WHITE) {
		zone = 6;
	} else {
		lane = comp_lane(currSensState.colorDetector);

		if(prevSensState.colorDetector == COLOR::WHITE)
			zone = 9;
	}

	for(int i = 0; i < Field::n_zones; ++i) {
		if(Field::zones_boxes[i].point_inside(drvbState.pos)) {
			zone = i;
			break;
		}
	}

	return { zone, lane };
}

Objective compute_knockCup_state(GameState const& gmState)
{
	// Knock cup (not valid because it will open way before and close way after)
	// &&Figureout&&
	if(gmState.missionState.knock_cup == Objective::Todo && gmState.zone >= 2) {
		return Objective::UnderWay;
	} else if(gmState.missionState.knock_cup == Objective::UnderWay && gmState.zone >= 6) {
		return Objective::Done;
	}
	return gmState.missionState.knock_cup;
}
Objective compute_one_cw_turn_state(GameState const& gmState,GameState const& previousGmState){
	if (gmState.missionState.one_cw_turn == Objective::Todo){
		return Objective::Start;
	}
	else if (gmState.missionState.one_cw_turn == Objective::Start){
		return Objective::UnderWay;
	}
	else if (gmState.missionState.one_cw_turn == Objective::UnderWay && gmState.zone == 9 && previousGmState.zone == 8 ){
		return Objective::Done;
	}
}
} // !p28