
#include "Drivebase.hpp"
#include <LibRobus.h>

namespace p28 {

void Drivebase::zero(SensorState currentSensState, SensorState prevSensState, Iteration_time it_time)
{
	update_kinematics(prevSensState.encoders_ticks, currentSensState.encoders_ticks, it_time.delta_s);
	update_wheels(mt::Vec2(0.0f), it_time.delta_s);
}
void Drivebase::update(SensorState currentSensState, SensorState prevSensState, Iteration_time it_time)
{
	update_kinematics(prevSensState.encoders_ticks, currentSensState.encoders_ticks, it_time.delta_s);

	if(finish) {
		update_wheels(mt::Vec2(0.0f), it_time.delta_s);
		return;
	}

	// if (drvMode == followLine){
		update_followLine(currentSensState, prevSensState, it_time);
	// }
	// else if(drvMode == followCam) {
		// update_followCam(currentSensState, prevSensState, it_time);
	// } else { // Follow path
		// update_followPath(it_time);
	// }
}
void Drivebase::update_kinematics(mt::i32Vec2 prevEncTicks, mt::i32Vec2 currEncTicks, float delta_s)
{
	// Do read
	// https://www.eecs.yorku.ca/course_archive/2017-18/W/4421/lectures/Wheeled%20robots%20forward%20kinematics.pdf

	mt::i32Vec2 ticks_diff = currEncTicks - prevEncTicks;
	wheelsVelocities = ticks_to_dist(ticks_diff) / delta_s;

	// Solving for velocity 
	// Page 11
	if(wheelsVelocities.left == wheelsVelocities.right) {
		trajectory_radius = kInfinity; // Not particulary a fan of div by 0
		angular_velocity = 0;
	} else {
		trajectory_radius = kRobotWidth_2 * (wheelsVelocities.right+wheelsVelocities.left) / (wheelsVelocities.right-wheelsVelocities.left);
		angular_velocity = (wheelsVelocities.right-wheelsVelocities.left) / kRobotWidth;
	}

	// Forward kinematics
	// page 20

	pos = pos + (wheelsVelocities.left+wheelsVelocities.right)/2.0f * heading * delta_s;
	heading = mt::normalize(mt::rotate(heading, 1/kRobotWidth * (wheelsVelocities.right - wheelsVelocities.left) * delta_s));
}

void Drivebase::update_followLine(SensorState currentSensState, SensorState prevSensState, Iteration_time it_time)
{
	char line = currentSensState.lineDetector;
	float dir = 0;
	for(int i = 0; i < 8; i++)
		{
			if(is_active(line, i))
			{
				// Serial.print("#");
				dir+=i-3.5;
			} else {
				// Serial.print(" ");
			}
		}
		// Serial.print(" => ");
		// Serial.print(dir);
		// Serial.println();
		if(line == 0)
		{
			finish = true;
		}
	//.02 is a magic number for the moment
	mt::Vec2 motorVel = mt::Vec2(dir, -dir)*.02 + kFollowLineBaseVel;
	update_wheels(motorVel, it_time.delta_s);
}

void Drivebase::update_followCam(SensorState currentSensState, SensorState prevSensState, Iteration_time it_time)
{
	// Serial.print("Block offset");
	// println(currentSensState.block_offset);
	mt::Vec2 motorVels;
	if(currentSensState.block_offset == mt::i32Vec2(0.0) && !currentSensState.block_in_claw) {
		motorVels = 0.0f;
	} else {
		float offset = (static_cast<float>(currentSensState.block_offset.x) / 40.0f);
		// offset = offset*offset*offset;
		float motorDelta =  offset * 0.08f;
		motorVels = mt::Vec2(-motorDelta, motorDelta);
	}

	if(currentSensState.block_offset.y < -2) {
		motorVels = -kFollowCamBaseVel;
	} else if(abs(currentSensState.block_offset.x) < 7) {
		motorVels += kFollowCamBaseVel;
	}


	// Serial.print("Motorvel ");
	// print(motorVel, 6);
	// Serial.println();
	update_wheels(motorVels, it_time.delta_s);
}

void Drivebase::update_followPath(Iteration_time it_time)
{
	if(path.finished())
	{
		Serial.println("Fiiinhished");
		finish=true;
		return;
	}
	Paths::CheckPoint checkPoint = path.current();

	// Go to next checkpoint in path if the current checkpoint is done
	if((!checkPoint.turn_only && mt::epsilon_equal2(pos, checkPoint.targPos, kPathFollower_distEpsilon2)) || 
		(checkPoint.turn_only && mt::epsilon_equal2(heading, checkPoint.targHeading, kPathFollower_headingEpsilon2))) {
		Serial.println("Neeext");
		path.index++;
		headingError = Error{};
		if(!path.finished()) {
			// path.current is not the same as current because we
			// just incremented the path!
			waitUntil_ms = it_time.time_ms + path.current().delay_before;
			checkPoint = path.current();
		} else {
			return;
		}
	}

	// Don't move if the drivebase should be waiting for actions	
	if(waitUntil_ms > it_time.time_ms || path.finished()) {
		update_wheels({0.0f}, it_time.delta_s);
		return;
	}

	if(checkPoint.turn_only) {
		update_turn(checkPoint, it_time);
	} else {
		update_follow_arc(checkPoint, it_time);
	}
}

void Drivebase::setDriveMode(Drivemodes mode)
{
	drvMode = mode;
	finish = false;
}

void Drivebase::set_path(Iteration_time it_time)
{
	if(!path.finished()) {
		setDriveMode(Drivemodes::followPath);
		waitUntil_ms = it_time.time_ms + path.current().delay_before;
	}
}
void Drivebase::update_follow_arc(Paths::CheckPoint follow, Iteration_time it_time)
{
	// 1. Find the arc that takes us from our current position to
	// the target position with a target heading
	Paths::Arc arc = Paths::arc_from_targetHeading(pos, follow.targPos, follow.targHeading);
	
	mt::Vec2 speed_correction = correct_heading();
	if(follow.backward) { 
		arc.radius = -arc.radius;
		// arc.tengeantStart = -arc.tengeantStart;
		speed_correction = -speed_correction;
	}

	// // If the angle between the current heading and the heading to be 
	// // tangeant to the arc is greater than 15 degrees, just turn
	// if(abs(mt::signed_angle(heading, arc.tengeantStart) > 0.267)) {
	// 	update_turn(Paths::CheckPoint::make_turn(arc.tengeantStart), it_time);
	// 	return;
	// }

	float targVel = velocity_for_point(velocity(), follow.targVel, follow.maxVel, arc.length, kAccel, it_time.delta_s);

	mt::Vec2 motor_speeds;
	if(arc.radius != kInfinity) {	
		// Transform m/s into rad/s
		float angular_vel = abs(targVel / arc.radius);


		// 3. Find the motor speeds needed to follow said arc, assuming
		motor_speeds = arc_to_motorVels(arc, angular_vel);
	} else {
		motor_speeds = { targVel, targVel };
	}
	// print(motor_speeds);
	motor_speeds += speed_correction;
	// Serial.print(" | ");
	// print(motor_speeds);
	// Serial.println();
	if(follow.backward) {
		update_wheels(-motor_speeds, arc.tengeantStart, it_time.delta_s);
	} else {
		update_wheels(motor_speeds, arc.tengeantStart, it_time.delta_s);
	}
}
void Drivebase::update_turn(Paths::CheckPoint follow, Iteration_time it_time)
{
	float err = abs(headingError.error);
	float motor_speed = KMinVel;
	if(err > PI/2) {
		motor_speed = 0.3;
	} else if(err > PI/4) {
		motor_speed = 0.1;
	}
	if(headingError.error < 0) {
		motor_speed = -motor_speed;
	}

	update_wheels({motor_speed, -motor_speed}, follow.targHeading, it_time.delta_s);
}


//makes the robot turn following a circular arc
mt::Vec2 Drivebase::arc_to_motorVels(Paths::Arc arc, float angular_vel)
{
	angular_vel = abs(angular_vel);
	// See https://www.eecs.yorku.ca/course_archive/2017-18/W/4421/lectures/Wheeled%20robots%20forward%20kinematics.pdf
	angular_vel = min(angular_vel, kMaxAngularVelocity); // Ensure we do not go over the maximum angular velocity
	mt::Vec2 speedBothMotor;
	speedBothMotor.left 	= abs(angular_vel * ( arc.radius - kRobotWidth_2 ));   // speed of the interior wheel in m/s 
	speedBothMotor.right 	= abs(angular_vel * ( arc.radius + kRobotWidth_2 ));   //speed of the exteriorwheel in m/s 
	return speedBothMotor;
}

mt::Vec2 Drivebase::correct_heading() const
{
	float velocity_offset = get(headingPID, headingError);
	return mt::Vec2(velocity_offset, -velocity_offset);
}


float Motor::hardware_output() const
{
	// Serial.print("P: ");
	// Serial.println(pid.P);
	return mt::clamp(get(pid, error), -1.0f, 1.0f);
}


float Drivebase::velocity()
{
	if(trajectory_radius == kInfinity) {
		return abs(wheelsVelocities.left); // Going in a perfect straigth line, both weels go at the drivebase velocity
	} else {
		return abs(angular_velocity * trajectory_radius);
	}
}

void Drivebase::update_wheels(mt::Vec2 target_wheelVels, double delta_s)
{
	leftWheel.error = update_error(leftWheel.error, wheelsVelocities.left, target_wheelVels.left, delta_s);
	rightWheel.error = update_error(rightWheel.error, wheelsVelocities.right, target_wheelVels.right, delta_s);
}
void Drivebase::update_wheels(mt::Vec2 target_wheelVels, mt::Vec2 target_heading, double delta_s)
{
	update_wheels(target_wheelVels, delta_s);

	float angle_error = mt::clamp(mt::signed_angle(heading, target_heading), -PI/2, PI/2);

	headingError = update_error(headingError, angle_error, 0.0, delta_s);
}

HardwareState Drivebase::aggregate(HardwareState hrdwState)
{
	hrdwState.motors = { leftWheel.hardware_output(), rightWheel.hardware_output() };
	return hrdwState;
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