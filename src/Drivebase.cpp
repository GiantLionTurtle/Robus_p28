#include "Drivebase.hpp"
#include "Field.hpp"
#include <LibRobus.h>
#include "ProximityDetector.hpp"

namespace p28 {

struct Line {
	mt::Vec2 origin;
	mt::Vec2 dir;
};

mt::Vec2 lineLine_intersection(Line const& l1, Line const& l2)
{
	mt::Vec2 originDiff = l1.origin - l2.origin;
	float dirCross = mt::cross(l1.dir, l2.dir);

	return l1.origin + l1.dir * mt::cross(l2.dir, originDiff) / dirCross;
}
// Are three points aranged in a ccw fashion?
bool threePoints_ccw(mt::Vec2 A, mt::Vec2 B, mt::Vec2 C)
{
	return (C.y-A.y) * (B.x-A.x) > (B.y-A.y) * (C.x-A.x);
}

PathSegment::PathSegment(DrivebaseState drvbState, p28::mt::Vec2 targPos_, float targSpeed_, p28::mt::Vec2 targHeading_)
	: targPos(targPos_)
	, targSpeed(targSpeed_)
{
	// Compute the arc that takes us from the current position 
	// to the target position at a heading

	// Step 1 compute radius
	mt::Vec2 currentToTarget = targPos - drvbState.pos;

	// Line1, perpendicular to the line from current pos to target pos
	Line line1 { drvbState.pos + currentToTarget/2.0f, 
				{-currentToTarget.y, currentToTarget.x} };

	// Line2, perpendicular to tangeant to the arc, going through the target position
	Line line2 { targPos, { -targHeading_.y, targHeading_.x } };

	mt::Vec2 circleCenter = lineLine_intersection(line1, line2);

	// Find a radius, if negative, it means the left wheel should go 
	// faster than the right wheel
	pathRadius = mt::magnitude(circleCenter - targPos);

	// Step 2 cw or ccw
	
	// is the target heading pointing cw or ccw
	if(!threePoints_ccw(circleCenter, targPos, targPos + targHeading_)) {
		pathRadius = -pathRadius;
	}
}
PathSegment::PathSegment(p28::mt::Vec2 targPos_, float targSpeed_)
	: targPos(targPos_)
	, targSpeed(targSpeed_)
	, pathRadius(10000000) // Arbitrary big radius (infinite)
{

}
PathSegment::PathSegment(p28::mt::Vec2 targPos_, float targSpeed_, float pathRadius_)
	: targPos(targPos_)
	, targSpeed(targSpeed_)
	, pathRadius(pathRadius_)
{

}

DrivebasePath DrivebasePath::update_path(DrivebaseState drvbState) const
{
	if(mt::epsilon_equal(drvbState.pos, current().targPos, kPathFollower_epsilon2)) {
		DrivebasePath out = *this;
		out.index++;
		if(index >= path.size())
			return {};
		return out;
	}
	return *this;
}


// Public functions
float ticks_to_dist(int32_t ticks)
{
	return static_cast<float>(ticks) / 3200 * TWO_PI * kWheelRadius;
}
float comp_accel_dist(float accel, float currSpeed, float targSpeed)
{
	return (targSpeed - currSpeed) / accel / 2.0;
}

// Motor get_motor_speed(struct Motor motor, float delta_s)
// {
// 	int32_t current_ticks = ENCODER_Read(motor.ID);
// 	int32_t ticks_diff = current_ticks - motor.last_ticks;
// 	motor.speed = ticks_to_dist(ticks_diff) / delta_s;
// 	motor.last_ticks = current_ticks;
// 	return motor;
// }
// Motor update_motor_at_speed(struct Motor motor, float set_speed, long int time_ms)
// {
// 	long int diff_time_ms = time_ms - motor.last_time_ms;
// 	float delta_s = static_cast<float>(diff_time_ms) / 1000.0f;

// 	motor.last_time_ms = time_ms;
// 	motor = get_motor_speed(motor, delta_s);
// 	motor.error = update_error(motor.error, motor.speed, set_speed, delta_s);
// 	float harware_set = get(motor.pid, motor.error);

// 	MOTOR_SetSpeed(motor.ID, harware_set);
// 	return motor;
// }

DrivebaseState DrivebaseState::update(mt::i32Vec2 prevEncTicks, mt::i32Vec2 currEncTicks, float delta_s) const
{
	// &&Figureout&&;
	return *this;
}

Pair<mt::Vec2, Drivebase> Drivebase::hardware_output(PathSegment const& follow, unsigned long time_ms, float delta_s) const
{
	if(state.waitUntil > time_ms)
		return { mt::Vec2(0.0), *this }; // Don't move if the drivebase should be waiting for actions

	// &&Figureout&& out how to follow a segment
	mt::Vec2 motor_speeds;

	return { motor_speeds, *this };
}

} // !p28