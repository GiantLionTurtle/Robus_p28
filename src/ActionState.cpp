
#include "ActionState.hpp"
#include "Drivebase.hpp"

namespace p28 {

DrivebasePath gen_ballSwervePath(Robot const& robState)
{

}
void ballSwerve_helper(ActionState& actState, Robot const& robState, Objective obj_state)
{
	if(obj_state == Objective::Start) {
		actState.path = gen_ballSwervePath(robState);
	} else if(obj_state == Objective::UnderWay && actState.path.index == kCupRelease_pathIndex) {
		actState.releaseCup = true;
	}
}

ActionState generate_actionState(ActionState prevActState, Robot robState, GameState gmState)
{
	ActionState actState;
	actState.path = prevActState.path.update_path(robState.drvb.state);

	// Cup zone?
	if(gmState.missionState.knock_cup == Objective::UnderWay) {
		actState.openArm = true;
	}

	if(gmState.missionState.trap_ball != Objective::Todo) {
		ballSwerve_helper(actState, robState, gmState.missionState.trap_ball);
	}

	return actState;
}

} // !p28