
#include "Drivebase.hpp"

#include "Constants.hpp"

namespace p28 {

namespace Drivebase {

float ticks_to_dist(long long int n_ticks)
{
	return static_cast<float>(n_ticks) / static_cast<float>(kTicksPerRotation) // Number of rotations
			* TWO_PI * kWheelRadius; // Distance per rotation
}

Desc kinematics(Desc prev, float dist_l, float dist_r, float delta_s)
{
	if(dist_r == dist_l) { // Unlikely but division by 0 is not fun
		return Desc(prev.pos + prev.front * dist_r, prev.front);
	}
	// Speed of each wheel
	float vel_r = dist_r / delta_s;
	float vel_l = dist_l / delta_s;

	// Angular velocity (rad / s)
	float angVel = (vel_r - vel_l) / kRobotWidth;

	// Radius of curvature (m) can be negative which means we are turning right
	float curvRadius = kRobotWidth_2 * (vel_r + vel_l) / (vel_r - vel_l);

	// Center of curvature
	mt::Vec2 left { -prev.front.y, prev.front.x };
	mt::Vec2 centerOfCurvature = left * curvRadius + prev.pos;

	// Radial movement (rad)
	float radMov = angVel * delta_s;

	// Offset the prev position by the center of curvature
	mt::Vec2 pos_offset = prev.pos - centerOfCurvature; // Vector center to prev pos
	mt::Vec2 newPos_offset = mt::rotate(pos_offset, radMov);

	Desc out;
	out.pos = newPos_offset + centerOfCurvature;
	out.front = mt::rotate(prev.front, radMov);

	return out;
}

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
bool is_vectorRightOfFront(mt::Vec2 const& vec, mt::Vec2 const& front)
{
	return mt::cross(front, vec) < 0.0;
}
mt::Vec2 invert_vec(mt::Vec2 const& vec)
{
	return { vec.y, vec.x };
}
mt::Vec2 inverse_kinematics(Desc current, mt::Vec2 target_pos, float maxVel)
{
	if(current.pos == target_pos) // Unlikely
		return mt::Vec2(0.0f);
	
	// Step1 
	// Find the circle that gets us from the current state of
	// the drivebase to a target position
	mt::Vec2 currentToTarget = target_pos - current.pos;

	// Line1 from the middle point of current and target
	// perpendicular that
	Line line1 { current.pos + currentToTarget/2.0f, 
				{-currentToTarget.y, currentToTarget.x} };

	// Line2
	Line line2 { current.pos, current.front };

	mt::Vec2 circleCenter = lineLine_intersection(line1, line2);

	// Find a radius, if negative, it means the left wheel should go 
	// faster than the right wheel
	mt::Vec2 currentToCenter = circleCenter - current.pos;
	double radius = mt::magnitude(currentToCenter);
	// Step2
	// Find the motor speeds to run in the circle
	mt::Vec2 out;
	out[kLeftMotor] 	= kMaxAngVelocity * (radius - kRobotWidth_2);
	out[kRightMotor] 	= kMaxAngVelocity * (radius + kRobotWidth_2);

	out = mt::normalize(out) * maxVel;

	// Swap the output if a the robot should actualy turn right
	if(is_vectorRightOfFront(currentToCenter, current.front))
		out = invert_vec(out);
	return out;
}

mt::Vec2 orient_toward(Desc prev, mt::Vec2 direction, float maxVel)
{
	mt::Vec2 out;
	out[kRightMotor] = maxVel;
	out[kLeftMotor] = -maxVel;
	if(is_vectorRightOfFront(direction, prev.front))
		out = invert_vec(out);
	return out;
}

} // !Drivebase

} // !p28