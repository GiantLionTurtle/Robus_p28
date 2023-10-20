#include "Drivebase.hpp"
#include <LibRobus.h>
#include "ProximityDetector.hpp"

namespace p28 {

mt::Vec2 Line::line_intersection(Line const& l2) const
{
	mt::Vec2 originDiff = origin - l2.origin;
	float dirCross = mt::cross(dir, l2.dir);

	return origin + dir * mt::cross(l2.dir, originDiff) / dirCross;
}
// Are three points aranged in a ccw fashion?
bool threePoints_ccw(mt::Vec2 A, mt::Vec2 B, mt::Vec2 C)
{
	return (C.y-A.y) * (B.x-A.x) > (B.y-A.y) * (C.x-A.x);
}

// PathSegment::PathSegment(DrivebaseState drvbState, mt::Vec2 targPos_, float targSpeed_, mt::Vec2 targHeading_)
// 	: targPos(targPos_)
// 	, targSpeed(targSpeed_)
// {
// 	// Compute the arc that takes us from the current position 
// 	// to the target position at a heading

// 	// Step 1 compute radius
// 	mt::Vec2 currentToTarget = targPos - drvbState.pos;

// 	// Line1, perpendicular to the line from current pos to target pos
// 	Line line1 { drvbState.pos + currentToTarget/2.0f, 
// 				{-currentToTarget.y, currentToTarget.x} };

// 	// Line2, perpendicular to tangeant to the arc, going through the target position
// 	Line line2 { targPos, { -targHeading_.y, targHeading_.x } };

// 	mt::Vec2 circleCenter = lineLine_intersection(line1, line2);

// 	// Find a radius, if negative, it means the left wheel should go 
// 	// faster than the right wheel
// 	pathRadius = mt::magnitude(circleCenter - targPos);

// 	// Step 2 cw or ccw
	
// 	// is the target heading pointing cw or ccw
// 	if(!threePoints_ccw(circleCenter, targPos, targPos + targHeading_)) {
// 		pathRadius = -pathRadius;
// 	}
// }

PathSegment::PathSegment(mt::Vec2 targPos_, mt::Vec2 targHeading_, float targSpeed_)
	: targPos(targPos_)
	, targHeading(targHeading_)
	, targSpeed(targSpeed_)
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
	return static_cast<float>(ticks) / kTicksPerRotation * TWO_PI * kWheelRadius;
}
float comp_accel_dist(float accel, float currSpeed, float targSpeed)
{
	return (targSpeed - currSpeed) / accel / 2.0;
}
mt::Vec2 ticks_to_dist(mt::i32Vec2 Ticks)
{
	return {ticks_to_dist(Ticks.left), ticks_to_dist(Ticks.right)};
}

mt::Vec2 get_motor_speed(mt::i32Vec2 prevEncTicks, mt::i32Vec2 currEncTicks, float delta_s)
{
	mt::i32Vec2 ticks_diff = currEncTicks - prevEncTicks;
	mt::Vec2 motor_speed = ticks_to_dist(ticks_diff) / delta_s;
	return motor_speed;
}
Pair<Motor, float> update_motor_at_speed(Motor motor, float set_speed,float actual_speed, float delta_s)
{
	motor.error = update_error(motor.error, actual_speed, set_speed, delta_s);
	float hardware_set = get(motor.pid, motor.error);

	return {motor, hardware_set};
}


DrivebaseState DrivebaseState::update(mt::i32Vec2 prevEncTicks, mt::i32Vec2 currEncTicks, float delta_s) const
{
	// &&Figureout&&; 
	DrivebaseState new_drvbState;
	new_drvbState.wheelsVelocities = get_motor_speed(prevEncTicks, currEncTicks, delta_s);
	return new_drvbState;
}

Pair<mt::Vec2, Drivebase> Drivebase::hardware_output(PathSegment const& follow, unsigned long time_ms, float delta_s) const
{
	if(state.waitUntil > time_ms)
		return { mt::Vec2(0.0), *this }; // Don't move if the drivebase should be waiting for actions

	// 1. Find the arc that takes us from our current position to
	// the target position with a target heading
	Arc arc = arc_from_targetHeading(state.pos, follow.targPos, follow.targHeading);

	// 2. Find the angular velocity that should be reached this iteration
	// Assum we are going in a straight line of length arc.length
	float velocity = velocity_for_point(state.velocity, follow.targSpeed, arc.length, kAccel);

	// Transform m/s into rad/s
	float angularVelocity = velocity / arc.radius;

	// 3. Find the motor speeds needed to follow said arc, assuming
	// we are already tangeant to it with angular velocity
	mt::Vec2 motor_speeds = arcTurnToDest(arc, angularVelocity);

	// 4. Correct for the heading error
	// &&Figureout&&

	return { motor_speeds, *this };
}

// Create an arc that can be followed by the robot
Arc arc_from_targetHeading(mt::Vec2 start, mt::Vec2 end, mt::Vec2 end_heading)
{
	// &&Figureout&&
	// Essentialy the code from the defunct PathSegment constructor
	// + length
}

// Gives a velocity that tries to follow a trapezoidal acceleration patern
float velocity_for_point(float current_velocity, float target_velocity, float dist_to_target, float allowed_accel)
{
	// &&Figureout&&
}
//makes the robot turn following a circular arc
mt::Vec2 arcTurnToDest(Arc arc, float angularVelocity)
{
	// See https://www.eecs.yorku.ca/course_archive/2017-18/W/4421/lectures/Wheeled%20robots%20forward%20kinematics.pdf
	angularVelocity = min(angularVelocity, kMaxAngularVelocity); // Ensure we do not go over the maximum angular velocity
	float leftWheel 	= abs(angularVelocity * ( arc.radius - kRobotWidth_2 ));   // speed of the interior wheel in m/s 
	float rightWheel 	= abs(angularVelocity * ( arc.radius + kRobotWidth_2 ));   //speed of the exteriorwheel in m/s 
	mt::Vec2 speedBothMotor = { rightWheel, leftWheel };
	return speedBothMotor;
}

} // !p28