#include "Drivebase.hpp"
#include <LibRobus.h>
#include "ProximityDetector.hpp"

namespace p28 {

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

void Arc::print() const
{
	mt::print(tengeantStart, 4);
	Serial.print(",  ");
	mt::print(end, 4);
	Serial.print(",  ");
	Serial.print(radius, 4);
	Serial.print(",  ");
	Serial.println(length, 4);
}

PathSegment::PathSegment(mt::Vec2 targPos_, mt::Vec2 targHeading_, float targSpeed_)
	: targPos(targPos_)
	, targHeading(mt::normalize(targHeading_))
	, targSpeed(targSpeed_)
{

}


float Motor::hardware_output() const
{
	// Serial.print("P: ");
	// Serial.println(pid.P);
	return get(pid, error);
}

float DrivebaseState::velocity() const
{
	if(trajectory_radius == kInfinity) {
		return wheelsVelocities.left; // Going in a perfect straigth line, both weels go at the drivebase velocity
	} else {
		return angular_velocity * trajectory_radius;
	}
}
DrivebaseState DrivebaseState::update_kinematics(mt::i32Vec2 prevEncTicks, mt::i32Vec2 currEncTicks, float delta_s) const
{
	// Do read
	// https://www.eecs.yorku.ca/course_archive/2017-18/W/4421/lectures/Wheeled%20robots%20forward%20kinematics.pdf

	DrivebaseState new_drvbState;
	mt::Vec2 wheelVels = get_motor_speed(prevEncTicks, currEncTicks, delta_s);
	new_drvbState.wheelsVelocities = wheelVels;
	new_drvbState.waitUntil = waitUntil; // Keep timer up


	// Solving for velocity 
	// Page 11
	if(wheelVels.left == wheelVels.right) {
		new_drvbState.trajectory_radius = kInfinity; // Not particulary a fan of div by 0
		new_drvbState.angular_velocity = 0;
	} else {
		new_drvbState.trajectory_radius = kRobotWidth_2 * (wheelVels.right+wheelVels.left) / (wheelVels.right-wheelVels.left);
		new_drvbState.angular_velocity = (wheelVels.right-wheelVels.left) / kRobotWidth;
	}

	// Forward kinematics
	// page 20

	new_drvbState.pos = pos + (wheelVels.left+wheelVels.right)/2.0f * heading * delta_s;
	new_drvbState.heading = mt::normalize(mt::rotate(heading, 1/kRobotWidth * (wheelVels.right - wheelVels.left) * delta_s));

	// Serial.print("Vel: ");
	// print(new_drvbState.wheelsVelocities);
	// Serial.print(" pos: ");
	// print(new_drvbState.pos);
	// Serial.print(" heading ");
	// print(heading);
	// Serial.print(" delta_s ");	
	// Serial.println(delta_s);

	return new_drvbState;
}
DrivebaseState DrivebaseState::intersect_line(mt::Line ln) const
{
	DrivebaseState out = *this;
	out.pos = ln.line_intersection(mt::Line{.origin=pos, .dir=heading});
	return out;
}
mt::Vec2 DrivebaseState::get_motor_speed(mt::i32Vec2 prevEncTicks, mt::i32Vec2 currEncTicks, float delta_s) const
{
	mt::i32Vec2 ticks_diff = currEncTicks - prevEncTicks;
	mt::Vec2 motor_speed = ticks_to_dist(ticks_diff) / delta_s;
	return motor_speed;
}

DrivebaseConcrete DrivebaseConcrete::update(mt::Vec2 actualWheelVelocities, mt::Vec2 desiredWheelVelocities, 
											mt::Vec2 currentHeading, mt::Vec2 targetHeading, Iteration_time it_time) const
{
	DrivebaseConcrete out = *this;
	out.left.error = update_error(left.error, actualWheelVelocities.left, desiredWheelVelocities.left, it_time.delta_s);
	out.right.error = update_error(right.error, actualWheelVelocities.right, desiredWheelVelocities.right, it_time.delta_s);

	// The heading error doesn't need to know the actual heading angle, only the
	// difference between target vector and current vector + goal is 0.0
	out.headingError = update_error(headingError, mt::signed_angle(currentHeading, targetHeading), 0.0, it_time.delta_s);

	return out;
}

mt::Vec2 DrivebaseConcrete::hardware_output() const
{
	return { left.hardware_output(), right.hardware_output() };
}
void Drivebase::update_path()
{
	if(path.index >= path.size)
		return;
	// mt::print(state.pos);
	// Serial.print(" vs ");
	// mt::print(path.current().targPos);
	// Serial.println("");
	if(mt::epsilon_equal2(state.pos, path.current().targPos, kPathFollower_epsilon2)) {

		Serial.println("Neeext");
		path.index++;
	}
}
void Drivebase::update_concrete(Iteration_time it_time)
{
	// Don't move if the drivebase should be waiting for actions	
	// Serial.print("Path size ");
	// Serial.print(path.size);
	// Serial.print(",  ");
	// Serial.println(path.index);
	if(state.waitUntil > it_time.time_ms || path.index >= path.size) {
		concrete = concrete.update(state.wheelsVelocities, {0.0f}, {1.0f}, {1.0f}, it_time);
		return;
	}

	PathSegment follow = path.current();
	// 1. Find the arc that takes us from our current position to
	// the target position with a target heading
	Arc arc = arc_from_targetHeading(state.pos, follow.targPos, follow.targHeading);
	// Arc arc { .tengeantStart=mt::Vec2(0.0, 1.0), .end=(1.0, 1.0), .radius=-1.0, .length=1.57 };

	// Serial.print("error: ");
	// Serial.println(error.error);
	// Serial.print(",  ");
	// Serial.print(error.sum_error);
	// Serial.print(",  ");
	// Serial.println(error.diff_error);


	// Serial.print("Pos: ");
	// print(state.pos);
	// Serial.println("");
	// Serial.print("Arc: ");
	// arc.print();
	// 2. Find the angular velocity that should be reached this iteration
	// Assum we are going in a straight line of length arc.length
	float velocity = velocity_for_point(state.velocity(), follow.targSpeed, arc.length, kAccel, it_time.delta_s);
	// Serial.print("Wants vel: ");
	// Serial.println(state.last_wanted_velocity);
	mt::Vec2 motor_speeds;
	if(arc.radius != kInfinity) {	
		// Transform m/s into rad/s
		float angularVelocity = min(abs(velocity / arc.radius), kMaxAngularVelocity);


		// 3. Find the motor speeds needed to follow said arc, assuming
		// we are already tangeant to it with angular velocity
		// Serial.print(angularVelocity);
		// Serial.print(" for ");
		// Serial.print(arc.length);
		// Serial.print(" :: ");
		// Serial.print(arc.radius);
		// Serial.print(" :: ");
		motor_speeds = arcTurnToDest(arc, angularVelocity);
	} else {
		motor_speeds = { velocity, velocity };
	}
	// Serial.print("Target velocity ");
	// print(motor_speeds);
	// Serial.print(" => ");
	// Serial.print(" => ");
	// Serial.print(velocity);
	// Serial.print(" from ");


	// 4. Correct for the heading error
	motor_speeds = correct_heading(motor_speeds);
	//mt::Vec2(0.2185, 0.1815)
	concrete = concrete.update(state.wheelsVelocities, motor_speeds, state.heading, arc.tengeantStart, it_time);
}
mt::Vec2 Drivebase::correct_heading(mt::Vec2 staged_wheelVelocities) const
{
	float velocity_offset = get(concrete.headingPID, concrete.headingError);
	return staged_wheelVelocities + mt::Vec2(velocity_offset, -velocity_offset);
}


// Create an arc that can be followed by the robot
Arc arc_from_targetHeading(mt::Vec2 start, mt::Vec2 end, mt::Vec2 end_heading)
{
	// See fig.3
	if(mt::epsilon_equal(end_heading, mt::normalize(end-start), 0.01f)) { // Protect against overflow and div by 0
		return Arc{ .tengeantStart=end_heading, .end=end, .radius=kInfinity, .length=mt::distance(end, start) };
	}

	Arc arcTGH;
	arcTGH.end = end;

	mt::Vec2 vecStartEnd = end - start;
	mt::Vec2 orientationCenterEnd = { end_heading.y,-end_heading.x };
	mt::Line line_end_center { .origin=end, .dir=orientationCenterEnd};
	mt::Line line_mid_startend_center {.origin=vecStartEnd/2.0f+start, .dir={-vecStartEnd.y, vecStartEnd.x}};

	mt::Vec2 center = line_end_center.line_intersection(line_mid_startend_center);

	mt::Vec2 center_to_end = end-center;
	mt::Vec2 center_to_start = start-center;

	float angleRadius_CenterEnd = angle(orientationCenterEnd,vecStartEnd);
	arcTGH.radius = (magnitude(vecStartEnd)/2)/(cos(angleRadius_CenterEnd));

	float circumference = 2*PI*abs(arcTGH.radius);
	float angleArc = mt::angle(center_to_start, center_to_end);

	arcTGH.tengeantStart = mt::normalize(mt::ccw_perpendicular(center_to_start));

	if(arcTGH.radius < 0.0f) { // If we are going clock wise
		angleArc = 2*PI-angleArc; // The angle is the complement of ccw angle
		arcTGH.tengeantStart = -arcTGH.tengeantStart; // Reverse start heading;		
	}

	arcTGH.length = circumference * (angleArc/(2*PI));
	return arcTGH;
}

// Gives a velocity that tries to follow a trapezoidal acceleration patern
float velocity_for_point(float current_velocity, float target_velocity, float dist_to_target, float allowed_accel, float delta_s)
{
	// See fig.1
	float decel = allowed_accel;
	float accel = allowed_accel-0.01;
	float velocity_diff = abs(target_velocity - current_velocity);
	float time_to_target_velocity = velocity_diff / decel;
	float dist_to_target_velocity = min(target_velocity, current_velocity) * time_to_target_velocity + // Zone A
									velocity_diff * (time_to_target_velocity) / 2; // Zone B

	if(dist_to_target >= dist_to_target_velocity) {
		// Serial.print("Accel ");
		// Serial.print(accel);
		// Serial.print(" * ");
		// Serial.println(delta_s);
		return min(current_velocity + accel * delta_s, kMaxVel);
	} else {
		// Serial.print("Decel ");
		// Serial.print(accel);
		// Serial.print(" * ");
		// Serial.println(delta_s);
		return max(current_velocity - decel * delta_s, target_velocity);
	}
}
//makes the robot turn following a circular arc
mt::Vec2 arcTurnToDest(Arc arc, float angularVelocity)
{
	// See https://www.eecs.yorku.ca/course_archive/2017-18/W/4421/lectures/Wheeled%20robots%20forward%20kinematics.pdf
	angularVelocity = min(angularVelocity, kMaxAngularVelocity); // Ensure we do not go over the maximum angular velocity
	mt::Vec2 speedBothMotor;
	speedBothMotor.left 	= abs(angularVelocity * ( arc.radius - kRobotWidth_2 ));   // speed of the interior wheel in m/s 
	speedBothMotor.right 	= abs(angularVelocity * ( arc.radius + kRobotWidth_2 ));   //speed of the exteriorwheel in m/s 
	return speedBothMotor;
}

// Public functions
float ticks_to_dist(int32_t ticks)
{
	return static_cast<float>(ticks) / kTicksPerRotation * TWO_PI * kWheelRadius;
}
mt::Vec2 ticks_to_dist(mt::i32Vec2 Ticks)
{
	return { ticks_to_dist(Ticks.left), ticks_to_dist(Ticks.right) };
}
int32_t dist_to_ticks(float dist)
{
	return dist / (TWO_PI * kWheelRadius) * kTicksPerRotation;
}
mt::i32Vec2 dist_to_ticks(mt::Vec2 dist)
{
	return { dist_to_ticks(dist.left), dist_to_ticks(dist.right) };
}

} // !p28