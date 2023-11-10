#include "Drivebase.hpp"
#include <LibRobus.h>
#include "ProximityDetector.hpp"

namespace p28 {



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
								float targVel_, bool backward_, 
								float maxVel_, unsigned int delay_before_)
	: targPos(targPos_)
	, targHeading(mt::normalize(targHeading_))
	, targVel(targVel_)
	, maxVel(maxVel_)
	, delay_before(delay_before_)
	, backward(backward_)
{
	
}
PathCheckPoint PathCheckPoint::make_turn(mt::Vec2 targHeading_, unsigned int delay_before)
{	
	PathCheckPoint out;
	// print(targHeading_);
	// print(mt::normalize(targHeading_));
	// Serial.println();
	out.targHeading = mt::normalize(targHeading_);
	out.turn_only = true;
	out.delay_before = 0;
	return out;
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
	return mt::clamp(get(pid, error), -1.0f, 1.0f);
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

	float angle_error = mt::clamp(mt::signed_angle(currentHeading, targetHeading), -PI/2, PI/2);

	if(dist_to_target < 0.05) {
		angle_error *= dist_to_target / 0.05;
	}
	// Serial.println(angle_error);
	// print(currentHeading);
	// Serial.print(" | ");
	// print(targetHeading);
	// Serial.println();
	out.headingError = update_error(headingError, angle_error, 0.0, it_time.delta_s);
	// Serial.print(it_time.time_ms);
	// Serial.print(",  ");
	// Serial.println(angle_error);

	return out;
}
DrivebaseConcrete DrivebaseConcrete::update(mt::Vec2 actualWheelVelocities, mt::Vec2 desiredWheelVelocities,
								 Iteration_time it_time) const{
	DrivebaseConcrete out = *this;
	out.left.error = update_error(left.error, actualWheelVelocities.left, desiredWheelVelocities.left, it_time.delta_s);
	out.right.error = update_error(right.error, actualWheelVelocities.right, desiredWheelVelocities.right, it_time.delta_s);
	return out;
}

mt::Vec2 DrivebaseConcrete::hardware_output() const
{
	return { left.hardware_output(), right.hardware_output() };
}
void Drivebase::update_path(Iteration_time it_time)
{
	if(path.finished())
		return;

	PathCheckPoint current = path.current();
	

	if((!current.turn_only && mt::epsilon_equal2(state.pos, current.targPos, kPathFollower_distEpsilon2)) || 
		(current.turn_only && mt::epsilon_equal2(state.heading, current.targHeading, kPathFollower_headingEpsilon2))) {
		Serial.println("Neeext");
		path.index++;
		concrete.headingError = Error{};
		if(!path.finished()) {
			// path.current is not the same as current because we
			// just incremented the path!
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
	if(state.waitUntil > it_time.time_ms || path.finished()) {
		concrete = concrete.update(state.wheelsVelocities, {0.0f}, {1.0f}, {1.0f}, 0.0, it_time);
		return;
	}

	PathCheckPoint follow = path.current();

	if(follow.turn_only) {
		update_turn(follow, it_time);
	} else {
		update_follow_arc(follow, it_time);
	}
}
void Drivebase::update_follow_arc(PathCheckPoint follow, Iteration_time it_time)
{
	// 1. Find the arc that takes us from our current position to
	// the target position with a target heading
	Arc arc = arc_from_targetHeading(state.pos, follow.targPos, follow.targHeading);
	
	mt::Vec2 speed_correction = correct_heading();
	if(follow.backward) { 
		arc.radius = -arc.radius;
		arc.tengeantStart = -arc.tengeantStart;
		speed_correction = -speed_correction;
	}
	// print(state.pos);
	// Serial.print(" | ");
	// print(state.heading);
	// Serial.print(" | ");
	// print(arc.end);
	// Serial.println();
	// arc.print();
	// If the angle between the current heading and the heading to be 
	// tangeant to the arc is greater than 15 degrees, just turn
	if(abs(mt::signed_angle(state.heading, arc.tengeantStart) > 0.267)) {
		update_turn(PathCheckPoint::make_turn(arc.tengeantStart), it_time);
		return;
	}

	float velocity = velocity_for_point(state.velocity(), follow.targVel, follow.maxVel, arc.length, kAccel, it_time.delta_s);

	mt::Vec2 motor_speeds;
	if(arc.radius != kInfinity) {	
		// Transform m/s into rad/s
		float angular_vel = abs(velocity / arc.radius);


		// 3. Find the motor speeds needed to follow said arc, assuming
		motor_speeds = arcTurnToDest(arc, angular_vel);
	} else {
		motor_speeds = { velocity, velocity };
	}
	// print(motor_speeds);
	motor_speeds += speed_correction;
	// Serial.print(" | ");
	// print(motor_speeds);
	// Serial.println();
	if(follow.backward) {
		concrete = concrete.update(state.wheelsVelocities, -motor_speeds, state.heading, arc.tengeantStart, arc.length, it_time);
	} else {
		concrete = concrete.update(state.wheelsVelocities, motor_speeds, state.heading, arc.tengeantStart, arc.length, it_time);
	}
}
void Drivebase::update_turn(PathCheckPoint follow, Iteration_time it_time)
{
	float err = abs(concrete.headingError.error);
	float motor_speed = KMinVel;
	if(err > PI/2) {
		motor_speed = 0.3;
	} else if(err > PI/4) {
		motor_speed = 0.1;
	}
	if(concrete.headingError.error < 0) {
		motor_speed = -motor_speed;
	}
	concrete = concrete.update(state.wheelsVelocities, {motor_speed, -motor_speed}, state.heading, follow.targHeading, kInfinity, it_time);
}

mt::Vec2 Drivebase::correct_heading() const
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

	if(abs(arcTGH.radius) >= kInfinity) {
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
float velocity_for_point(float current_vel, float target_vel, float max_vel, float target_dist, float accel, float delta_s)
{
	// See fig.1
	float decel = accel-0.01; // Slightly gentler decel

	float vel_diff = abs(target_vel - current_vel);
	float time_to_target_vel = vel_diff / decel;

	// Distance to reach the targe velocity if we were to accel/decel right now
	float dist_to_target_vel = min(target_vel, current_vel) * time_to_target_vel + // Zone A
									vel_diff * (time_to_target_vel) / 2; // Zone B

	if(target_dist >= dist_to_target_vel) {
		return min(current_vel + accel * delta_s, max_vel);
	} else {
		return max(current_vel - decel * delta_s, target_vel);
	}
}
//makes the robot turn following a circular arc
mt::Vec2 arcTurnToDest(Arc arc, float angular_vel)
{
	angular_vel = abs(angular_vel);
	// See https://www.eecs.yorku.ca/course_archive/2017-18/W/4421/lectures/Wheeled%20robots%20forward%20kinematics.pdf
	angular_vel = min(angular_vel, kMaxAngularVelocity); // Ensure we do not go over the maximum angular velocity
	mt::Vec2 speedBothMotor;
	speedBothMotor.left 	= abs(angular_vel * ( arc.radius - kRobotWidth_2 ));   // speed of the interior wheel in m/s 
	speedBothMotor.right 	= abs(angular_vel * ( arc.radius + kRobotWidth_2 ));   //speed of the exteriorwheel in m/s 
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

void Drivebase::update(SensorState currentSensState, SensorState prevSensState, Iteration_time it_time)
{
	state = state.update_kinematics(prevSensState.encoders_ticks, currentSensState.encoders_ticks, it_time.delta_s);
	if (followLine){
		updateFollowLine(currentSensState,prevSensState,it_time);
	}
	else {
		
	}
}
void Drivebase::updateFollowLine(SensorState currentSensState, SensorState prevSensState, Iteration_time it_time)
{
	char line = currentSensState.lineDetector;
	float dir = 0;
	for(int i = 0; i < 8; i++)
		{
			if(is_active(line, i))
			{
				Serial.print("#");
				dir+=i-3.5;
			} else {
				Serial.print(" ");
			}
		}
		Serial.print(" => ");
		Serial.print(dir);
		Serial.println();
	//.1 is a magic number for the moment
	mt::Vec2 motorVel = mt::Vec2(-dir, dir)*.02 + kFollowLineBaseVelocity;
	concrete = concrete.update(state.wheelsVelocities, mt::Vec2(0), it_time);
}
} // !p28