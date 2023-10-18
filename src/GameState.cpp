
#include "GameState.hpp"

namespace p28 {

int lane(COLOR color)
{
	switch(color) {
	case COLOR::BLUE: 	return 0;
	case COLOR::GREEN: 	return 1;
	case COLOR::YELLOW: return 2;
	case COLOR::RED: 	return 3;
	default: 			return -1;
	}
}
Pair<RobotState, GameState> compute_robotGame_state(
								SensorState prevSensState, SensorState currSensState, 
								RobotState robState, GameState gmState)
{
	if(currSensState.colorDetector == static_cast<int>(COLOR::BLACK)) {
		gmState.zone = 2;
	} else if(currSensState.colorDetector == static_cast<int>(COLOR::WHITE) 
			&& prevSensState.colorDetector != static_cast<int>(COLOR::WHITE)) {
		gmState.zone = 6;
	} else {
		gmState.lane = lane(static_cast<COLOR>(currSensState.colorDetector));

		if(prevSensState.colorDetector == static_cast<int>(COLOR::WHITE))
			gmState.zone = 9;
	}


	// Knock cup (not valid because it will open way before and close way after)
	if(gmState.missionState.knock_cup == Objective::Todo && gmState.zone >= 2) {
		gmState.missionState.knock_cup = Objective::UnderWay;
	} else if(gmState.missionState.knock_cup == Objective::UnderWay && gmState.zone >= 6) {
		gmState.missionState.knock_cup = Objective::Done;
	}

    return Pair<RobotState, GameState>(robState, gmState);
}

} // !p28