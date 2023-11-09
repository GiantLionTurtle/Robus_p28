
#include "GameState.hpp"
#include "Constants.hpp"
#include "Field.hpp"
#include "sensors.hpp"
#include "CompileFlags.hpp"


namespace p28 {

GameState GameState::initial(SensorState sensState)
{
	GameState initial_gameState;
	
	return initial_gameState;
}

GameState GameState::generate_next(SensorState prevSensState, SensorState currSensState, DrivebaseState drvbState,Iteration_time it_time) const
{
	GameState newGmState = *this;
}

} // !p28