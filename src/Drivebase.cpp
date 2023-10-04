#include "Drivebase.hpp"
#include "Field.hpp"
#include <LibRobus.h>
#include "ProximityDetector.hpp"

namespace p28 {

// Helper functions

/**
 * @brief updates the position (in meters) of the robot
 * given a distance travelled along a field direction
 * 
 * @param dist Distance that the robot just travelled
 * @param direction Direction in which the robot travelled
 * @param x
 * x position of the robot on a 2D plane 
 * @param y 
 * y position of the robot on a 2D plane
 */
void update_pos(float dist, int direction, float& x, float& y);
// move; LEFT or RIGHT, changes the robot direction
// direction is !!! RELATIVE TO THE FIELD !!! 
void update_orientation(int move, int& direction);


// Public functions

float ticks_to_dist(int32_t ticks)
{
	return static_cast<float>(ticks) / 3200 * TWO_PI * p28::kWheelRadius;
}

Motor get_motor_speed(Motor motor, float delta_s)
{
	int32_t current_ticks = ENCODER_Read(motor.ID);
	int32_t ticks_diff = current_ticks - motor.last_ticks;
	motor.speed = ticks_to_dist(ticks_diff) / delta_s;
	motor.last_ticks = current_ticks;
	return motor;
}
Motor update_motor_at_speed(Motor motor, float set_speed, long int time_ms)
{
	long int diff_time_ms = time_ms - motor.last_time_ms;
	float delta_s = static_cast<float>(diff_time_ms) / 1000.0f;

	motor.last_time_ms = time_ms;
	motor = get_motor_speed(motor, delta_s);
	motor.error = update_error(motor.error, motor.speed, set_speed, delta_s);
	float harware_set = get(motor.pid, motor.error);

	MOTOR_SetSpeed(motor.ID, harware_set);
	return motor;
}

/**
 * @brief updates the position (in meters) of the robot
 * given a distance travelled along a field direction
 * 
 * @param dist Distance that the robot just travelled
 * @param direction Direction in which the robot travelled (field centric)

 * @param x (out)
 * x position of the robot on a 2D plane 
 * @param y  (out)
 * y position of the robot on a 2D plane
 */
Drivebase update_pos(Drivebase drvb, float dist, int direction)
{
	switch(direction)
	{
		case FRONT:
			drvb.y+=dist;
			break;
		case REAR:
			drvb.y-=dist;
			break;
		case LEFT:
			drvb.x-=dist;
			break;
		case RIGHT:
			drvb.x+=dist;
			break;
		default:
			break;
	}

	drvb.sq_x = drvb.x / kSquareSize;
	drvb.sq_y = drvb.y / kSquareSize;
	return drvb;
}

// Move is field centric, orientation is robot centric
// @param orientation (out)
Drivebase update_orientation(Drivebase drvb, int move)
{
	if(move == LEFT) {
		switch(drvb.orientation)
		{
			case FRONT:
				drvb.orientation = LEFT;
				break;
			case REAR:
				drvb.orientation = RIGHT;
				break;
			case LEFT:
				drvb.orientation = REAR;
				break;
			case RIGHT:
				drvb.orientation = FRONT;
				break;
		}
	} else if(move == RIGHT) {
		switch(drvb.orientation)
		{
			case FRONT:
				drvb.orientation = RIGHT;
				break;
			case REAR:
				drvb.orientation = LEFT;
				break;
			case LEFT:
				drvb.orientation = FRONT;
				break;
			case RIGHT:
				drvb.orientation = REAR;
				break;
		}
	}
	return drvb;
}

Drivebase forward_dist(Drivebase drvb, float dist, float speed)
{
	long int init_ticks = drvb.left.last_ticks;
	long int init_time_ms = millis();
	float distance_parcourue = 0;
	drvb = set_motorTime(drvb, init_time_ms);

	while(distance_parcourue < dist) {
		delay(kControlLoopDelay);

		long int time_ms = millis();
		drvb.left = update_motor_at_speed(drvb.left, speed, time_ms);
		drvb.right = update_motor_at_speed(drvb.right, speed, time_ms);
		distance_parcourue = abs(ticks_to_dist(drvb.left.last_ticks-init_ticks));
	}

	double dist_mult = speed < 0 ? -1.0 : 1.0;
	drvb = update_pos(drvb, dist_mult*distance_parcourue, drvb.orientation);
	return zero_all(drvb);
}
Drivebase forward_until_detect(Drivebase drvb, float dist, float speed, float& traveled_dist, bool& detection)
{
	long int init_ticks = drvb.left.last_ticks;
	traveled_dist = 0;
	drvb = set_motorTime(drvb, millis());

	while(!detection && traveled_dist < dist)
	 {
		delay(kControlLoopDelay);
		long int time_ms = millis();
		drvb.left = update_motor_at_speed(drvb.left, speed, time_ms);
		drvb.right = update_motor_at_speed(drvb.right, speed, time_ms);
		traveled_dist = abs(ticks_to_dist(drvb.left.last_ticks-init_ticks));
		detection = wall_detection();
	}
	drvb = update_pos(drvb, traveled_dist, drvb.orientation);
	return zero_all(drvb);
}
Drivebase turn_right(Drivebase drvb)
{
	float dist_to_travel = PI * kRobotWidth / 4.0;
	long int init_ticks = drvb.right.last_ticks;
	drvb = set_motorTime(drvb, millis());

	while(abs(ticks_to_dist(drvb.right.last_ticks-init_ticks)) < dist_to_travel) {
		delay(kControlLoopDelay);

		long int time_ms = millis();
		drvb.left = update_motor_at_speed(drvb.left, 0.2, time_ms);
		drvb.right = update_motor_at_speed(drvb.right, -0.2, time_ms);
	}		

	drvb = update_orientation(drvb, RIGHT);
	return zero_all(drvb);
}
Drivebase turn_left(Drivebase drvb)
{
	float dist_to_travel = PI * kRobotWidth / 4.0;
	long int init_ticks = drvb.right.last_ticks;
	drvb = set_motorTime(drvb, millis());

	while(abs(ticks_to_dist(drvb.right.last_ticks-init_ticks)) < dist_to_travel) {
		delay(kControlLoopDelay);

		long int time_ms = millis();
		drvb.left = update_motor_at_speed(drvb.left, -0.2, time_ms);
		drvb.right = update_motor_at_speed(drvb.right, 0.2, time_ms);
	}
	drvb = update_orientation(drvb, LEFT);
	return zero_all(drvb);
}

// Given en orientation, is it fastest to turn left or right
// to point toward a direction?
bool is_fastest_left(int orientation, int target_direction){
	bool inv = false;
	if(target_direction < orientation){
		int tmp = target_direction;
		target_direction = orientation;
		orientation = tmp;
		inv = true;
	}
	bool out = true;
	
	if(orientation == LEFT && target_direction == FRONT)
		out = false;
	if(orientation == RIGHT && target_direction == REAR)
		out = false;
	
	if(inv)
		return !out;
	return out;
}

Drivebase move_to_square(Drivebase drvb, int direction, int n_squares)
{
	drvb = orient_toward_direction(drvb, direction);
	delay(kDecelerationDelay);
	drvb = forward_dist(drvb, kSquareSize, 0.2);

	return drvb;
}
Drivebase move_to_square_or_detect(Drivebase drvb, int direction, bool& detection)
{
	drvb = orient_toward_direction(drvb, direction);
	delay(kDecelerationDelay);
	float traveled_dist;
	drvb = forward_until_detect(drvb, kSquareSize/2.0, 0.2, traveled_dist, detection);

	if(detection) { // There was a wall
		delay(kDecelerationDelay);
		drvb = forward_dist(drvb, traveled_dist, -0.2);
	} else {
		drvb = forward_dist(drvb, kSquareSize-traveled_dist, 0.2);
	}
	return drvb;
}
Drivebase orient_toward_direction(Drivebase drvb, int direction)
{
	if(is_fastest_left(drvb.orientation, direction)) {
		while(direction != drvb.orientation) {
			drvb = turn_left(drvb);
		}
	} else {
		while(direction != drvb.orientation) {
			drvb = turn_right(drvb);
		}
	}
	return drvb;
}

Drivebase zero_all(Drivebase drvb)
{
	MOTOR_SetSpeed(LEFT, 0.0);
	MOTOR_SetSpeed(RIGHT, 0.0);

	drvb.left.speed = 0.0;
	drvb.right.speed = 0.0;
	return drvb;
}
Drivebase set_motorTime(Drivebase drvb, long int time_ms)
{
	drvb.left.last_time_ms = time_ms;
	drvb.right.last_time_ms = time_ms;
	return drvb;
}

} // !p28