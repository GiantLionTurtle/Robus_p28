
#include "GameState.hpp"
#include "Constants.hpp"

namespace p28 {

Pair<int, int> compute_zoneLane(SensorState  const& prevSensState, SensorState const&  currSensState, GameState const&  gmState);
Objective compute_knockCup_state(GameState const& gmState);

GameState GameState::next(SensorState prevSensState, SensorState currSensState) const
{
	GameState newGmState = *this;

	// Figure out where we are in game zones
	tie(newGmState.zone, newGmState.lane) = compute_zoneLane(prevSensState, currSensState, newGmState);

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

Pair<int, int> compute_zoneLane(SensorState const& prevSensState, SensorState const& currSensState, GameState const& gmState)
{
	int zone = gmState.zone;
	int lane = gmState.lane;
	if(currSensState.colorDetector == static_cast<int>(COLOR::BLACK)) {
		zone = 2;
	} else if(currSensState.colorDetector == static_cast<int>(COLOR::WHITE) 
			&& prevSensState.colorDetector != static_cast<int>(COLOR::WHITE)) {
		zone = 6;
	} else {
		lane = comp_lane(static_cast<COLOR>(currSensState.colorDetector));

		if(prevSensState.colorDetector == static_cast<int>(COLOR::WHITE))
			zone = 9;
	}
	return { zone, lane };
}

Objective compute_knockCup_state(GameState const& gmState)
{
	// Knock cup (not valid because it will open way before and close way after)
	if(gmState.missionState.knock_cup == Objective::Todo && gmState.zone >= 2) {
		return Objective::UnderWay;
	} else if(gmState.missionState.knock_cup == Objective::UnderWay && gmState.zone >= 6) {
		return Objective::Done;
	}
	return gmState.missionState.knock_cup;
}

} // !p28