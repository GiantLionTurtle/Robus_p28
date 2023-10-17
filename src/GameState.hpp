
#ifndef P28_GAMESTATE_HPP_
#define P28_GAMESTATE_HPP_

struct MissionState {
	// Mission objectives if the robot is not in race mode
    bool must_knock_glass { true };
    bool must_trap_ball { true };
    bool must_do_one_turn { true };
    bool must_do_one_shortcut_turn { true };

	// If >= 0, the current step (arc, line) to swerve around the ping pong ball
	int swerve_ball_step { -1 };
};

struct GameState {
	int zone; // The zone the robot is currently in
	bool in_race; // In race or in qualifications

	// How the mission is going (if not in race mode)
	MissionState missionState;
};

#endif