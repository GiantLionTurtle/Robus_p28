#include "Drivebase.hpp"
#include <LibRobus.h>
#include "ProximityDetector.hpp"

namespace p28 {

template<typename T>
T clamp(T x, T min_, T max_)
{
	if(x < min_)
		return min_;
	if(x > max_)
		return max_;
	return x;
}

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

PathCheckPoint::PathCheckPoint(mt::Vec2 targPos_, mt::Vec2 targHeading_, 
								float targSpeed_, unsigned int delay_before_, bool backward_)
	: targPos(targPos_)
	, targHeading(mt::normalize(targHeading_))
	, targSpeed(targSpeed_)
	, delay_before(delay_before_)
	, backward(backward_)
{
	
}
void DrivebasePath::add_checkPoint(PathCheckPoint checkPoint)
{
	if(size >= kMaxCheckPointForPath)
		return;
	segments[size] = checkPoint;
	size++;
}

float Motor::hardware_output() const
{
	// Serial.print("P: ");
	// Serial.println(pid.P);
	return clamp(get(pid, error), -1.0f, 1.0f);
}

float DrivebaseState::velocity() const
{
	if(trajectory_radius == kInfinity) {
		return abs(wheelsVelocities.left); // Going in a perfect straigth line, both weels go at the drivebase velocity
	} else {
		return abs(angular_velocity * trajectory_radius);
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
											mt::Vec2 currentHeading, mt::Vec2 targetHeading, float dist_to_target, Iteration_time it_time) const
{
	DrivebaseConcrete out = *this;
	out.left.error = update_error(left.error, actualWheelVelocities.left, desiredWheelVelocities.left, it_time.delta_s);
	out.right.error = update_error(right.error, actualWheelVelocities.right, desiredWheelVelocities.right, it_time.delta_s);

	// Serial.print("desired vs actual ");
	// print(desiredWheelVelocities);
	// Serial.print(" :: ");
	// print(actualWheelVelocities);
	// Serial.println();

	// The heading error doesn't need to know the actual heading angle, only the
	// difference between target vector and current vector + goal is 0.0

	float angle_error = mt::signed_angle(currentHeading, targetHeading);

	if(dist_to_target < 0.05) {
		angle_error *= dist_to_target / 0.05;
	}
	out.headingError = update_error(headingError, angle_error, 0.0, it_time.delta_s);
	// Serial.print(it_time.time_ms);
	// Serial.print(",  ");
	// Serial.println(angle_error);

	return out;
}

mt::Vec2 DrivebaseConcrete::hardware_output() const
{
	// Serial.print("left: ");
	// Serial.print(left.error.error);
	// Serial.print(",  ");
	// Serial.print(left.error.sum_error);
	// Serial.print(",  ");
	// Serial.println(left.error.diff_error);
	return { left.hardware_output(), right.hardware_output() };
}
void Drivebase::update_path(Iteration_time it_time)
{
	if(path.finished())
		return;
	// mt::print(state.pos);
	// Serial.print(" vs ");
	// mt::print(path.current().targPos);
	// Serial.println("");
	if(mt::epsilon_equal2(state.pos, path.current().targPos, kPathFollower_epsilon2)) {
		Serial.println("Neeext");
		path.index++;
		concrete.headingError = Error{};
		if(!path.finished()) {
			state.waitUntil = it_time.time_ms + path.current().delay_before;
		}
	}
}
void Drivebase::set_path(DrivebasePath path_, Iteration_time it_time)
{
	path = path_;
	if(!path.finished()) {
		state.waitUntil = it_time.time_ms + path.current().delay_before;
	}
}
void Drivebase::update_concrete(Iteration_time it_time)
{
	// Don't move if the drivebase should be waiting for actions	
	// Serial.print("Path size ");
	// Serial.print(path.size);
	// Serial.print(",  ");
	// Serial.println(path.index);
	if(state.waitUntil > it_time.time_ms || path.finished()) {
		concrete = concrete.update(state.wheelsVelocities, {0.0f}, {1.0f}, {1.0f}, 0.0, it_time);
		return;
	}

	PathCheckPoint follow = path.current();
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
	Serial.print("Arc: ");
	arc.print();
	// 2. Find the angular velocity that should be reached this iteration
	// Assum we are going in a straight line of length arc.length
	// float mod_accel = 1 - abs(concrete.headingError.error / PI);
	float velocity = velocity_for_point(state.velocity(), follow.targSpeed, arc.length, kAccel, it_time.delta_s);
	// Serial.print("Wants vel: ");
	// Serial.print(velocity);
	Serial.print(" actual: ");
	Serial.print(state.velocity());
	// Serial.print(" correction: ");
	mt::Vec2 motor_speeds;
	if(arc.radius != kInfinity) {	
		// Transform m/s into rad/s
		float angularVelocity = abs(velocity / arc.radius);


		// 3. Find the motor speeds needed to follow said arc, assuming
		// we are already tangeant to it with angular velocity
		// Serial.print("ang vel: ");
		// Serial.println(angularVelocity);
		// Serial.print(" for ");
		// Serial.print(arc.length);
		// Serial.print(" :: ");
		// Serial.print(arc.radius);
		// Serial.print(" :: ");
		motor_speeds = arcTurnToDest(arc, angularVelocity);
	} else {
		motor_speeds = { velocity, velocity };
	}
	Serial.print("Target velocity ");
	print(motor_speeds);
	Serial.print(" => ");
	// Serial.print(" => ");
	// Serial.print(velocity);
	// Serial.print(" from ");


	// 4. Correct for the heading error
	mt::Vec2 speed_correction = correct_heading(motor_speeds);
	// print(speed_correction);
	Serial.println();
	motor_speeds += speed_correction;//static_cast<float>(pow(state.velocity() / kMaxVel, 2));

	//mt::Vec2(0.2185, 0.1815)
	if(follow.backward) {
		concrete = concrete.update(state.wheelsVelocities, -motor_speeds, state.heading, -arc.tengeantStart, arc.length, it_time);
	} else {
		concrete = concrete.update(state.wheelsVelocities, motor_speeds, state.heading, arc.tengeantStart, arc.length, it_time);
	}
}
mt::Vec2 Drivebase::correct_heading(mt::Vec2 staged_wheelVelocities) const
{
	float velocity_offset = get(concrete.headingPID, concrete.headingError);
	return mt::Vec2(velocity_offset, -velocity_offset);
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

	// Serial.print("Radius ");
	// Serial.println(arcTGH.radius);
	if(abs(arcTGH.radius) >= kInfinity) {
		// Serial.println("Is infiinitylpol");
		return Arc{ .tengeantStart=end_heading, .end=end, .radius=kInfinity, .length=mt::distance(end, start) };
	}

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

	// Serial.print("dists: ");
	// Serial.print(dist_to_target);
	// Serial.print(",  ");
	// Serial.print(dist_to_target_velocity);
	// Serial.print(" with ");
	// Serial.print(current_velocity);
	// Serial.print(" :: ");
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
	angularVelocity = abs(angularVelocity);
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