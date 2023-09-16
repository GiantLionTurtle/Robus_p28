
#ifndef P28_DRIVEBASE_HPP_
#define P28_DRIVEBASE_HPP_

#include <Utils/Vec.hpp>

namespace p28 {

namespace Drivebase {

/*
	@struct Desc describes the position 
	of the drivebase, it has a position
	and a heading
*/
struct Desc {
	mt::Vec2 pos { 0.0 }; // Position in m
	mt::Vec2 front { 0.0, 1.0 }; // Normalized front vector, by default the robot looks toward the y axis

	Desc() = default;
	Desc(mt::Vec2 const& pos_, mt::Vec2 const& front_)
		: pos(pos_)
		, front(front_)
	{

	}
};

// Converts a number of encoder ticks to a distance in meters
float ticks_to_dist(long long int n_ticks);

// Returns a new drivebase desc from a current desc, the distance traveled
// by both wheels and the time interval between the current desc and the present
Desc kinematics(Desc prev, float dist_l, float dist_r, float delta_s);

// Returns the speed of both wheels to go from the current desc to a pos
// with a maximum velocity.
// Not smart enough to orient itself with a desc end
// It is expected that the target pos is rather close to the robot
// and be fed by a path follower algorithm of some sort
mt::Vec2 inverse_kinematics(Desc prev, mt::Vec2 target_pos, float maxVel);

// Returns the speed of both wheels to get to a direction
// it must be managed by an external path follower 
mt::Vec2 orient_toward(Desc prev, mt::Vec2 direction, float maxVel);

} // !Drivebase

} // !p28

#endif