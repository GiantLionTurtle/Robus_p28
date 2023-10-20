
#include "ActionState.hpp"
#include "Drivebase.hpp"

namespace p28 {

DrivebasePath gen_ballSwervePath(Robot const& robState)
{
	// &&Figureout&&
}
void ballSwerve_helper(ActionState& actState, Robot const& robState, Objective obj_state)
{
	if(obj_state == Objective::Start) {
		actState.path = gen_ballSwervePath(robState);
	} else if(obj_state == Objective::UnderWay && actState.path.index == kCupRelease_pathIndex) {
		actState.releaseCup = true;
	}
}

ActionState ActionState::generate_next(Robot robState, GameState gmState) const
{
	ActionState actState;
	actState.path = path.update_path(robState.drvb.state);

	// &&Figureout&& how to hook the drivebase code here

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