#include "Drivebase.hpp"
#include "Field.hpp"
#include <LibRobus.h>
#include "ProximityDetector.hpp"
#include "TraveledPath.hpp"


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
	return static_cast<float>(ticks) / 3200 * TWO_PI * kWheelRadius;
}
float comp_accel_dist(float accel, float currSpeed, float targSpeed)
{
	return (targSpeed - currSpeed) / accel;
}

struct Motor get_motor_speed(struct Motor motor, float delta_s)
{
	int32_t current_ticks = ENCODER_Read(motor.ID);
	int32_t ticks_diff = current_ticks - motor.last_ticks;
	motor.speed = ticks_to_dist(ticks_diff) / delta_s;
	motor.last_ticks = current_ticks;
	return motor;
}
struct Motor update_motor_at_speed(struct Motor motor, float set_speed, long int time_ms)
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
struct Drivebase update_pos(struct Drivebase drvb, float dist, int direction)
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
struct Drivebase update_orientation(struct Drivebase drvb, int move)
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

float do_accell(float curr, float accel, float delta_s)
{
	return curr + accel * delta_s;
}
float next_speed(float curr_speed, float target, float accel, float curr_dist, float accel_dist, float decel_dist, float delta_s)
{
	if(target < 0.0) {
		return -next_speed(-curr_speed, -target, -accel, curr_dist, accel_dist, decel_dist, delta_s);
	}
	if(curr_dist < accel_dist) {
		// Serial.println("accel");
		return min(do_accell(curr_speed, accel, delta_s), target);
	} else if(curr_dist > decel_dist) {
		// Serial.println("decl");
		return max(do_accell(curr_speed, -accel, delta_s), kMinSpeed);
	} else {
		// Serial.println("Stable");
		return target;
	}

}
void accel_decel_dist(float target_speed, float accel, float max_dist, float& accel_dist, float& decel_dist)
{
	accel_dist = abs(comp_accel_dist(accel, 0.0, target_speed));
	if(max_dist < 2*accel_dist) {
		accel_dist = max_dist/2.0;
	}
	decel_dist = max_dist - accel_dist;
}
struct Drivebase forward_dist(struct Drivebase drvb, float dist, float speed)
{
	long int init_ticks = drvb.left.last_ticks;
	long int init_time_ms = millis();
	float distance_parcourue = 0;
	drvb = set_motorTime(drvb, init_time_ms);

	float curr_speed = 0.0;
	float accel_dist, decel_dist;
	accel_decel_dist(speed, kAccel, dist, accel_dist, decel_dist);
	float dist_mult = speed < 0 ? -1.0 : 1.0;

	while(distance_parcourue < dist) {
		delay(kControlLoopDelay);

		long int time_ms = millis();

		long int diff_time_ms = time_ms - drvb.left.last_time_ms;
		float delta_s = static_cast<float>(diff_time_ms) / 1000.0f;
		curr_speed = dist_mult*next_speed(abs(curr_speed), abs(speed), kAccel, distance_parcourue, accel_dist, decel_dist, delta_s);

		drvb.left = update_motor_at_speed(drvb.left, curr_speed, time_ms);
		drvb.right = update_motor_at_speed(drvb.right, curr_speed, time_ms);
		distance_parcourue = abs(ticks_to_dist(drvb.left.last_ticks-init_ticks));
	}

	drvb = update_pos(drvb, dist_mult*distance_parcourue, drvb.orientation);
	return zero_all(drvb);
}
struct Drivebase forward_and_detect(struct Drivebase drvb, float dist, float speed, float& traveled_dist, bool& detection)
{
	long int init_ticks = drvb.left.last_ticks;
	traveled_dist = 0;
	drvb = set_motorTime(drvb, millis());

	float curr_speed = 0.0;
	float accel_dist, decel_dist;
	accel_decel_dist(speed, kAccel, dist, accel_dist, decel_dist);

	while(traveled_dist < dist) {
		delay(kControlLoopDelay);
		long int time_ms = millis();

		long int diff_time_ms = time_ms - drvb.left.last_time_ms;
		float delta_s = static_cast<float>(diff_time_ms) / 1000.0f;
		curr_speed = next_speed(curr_speed, speed, kAccel, traveled_dist, accel_dist, decel_dist, delta_s);

		drvb.left = update_motor_at_speed(drvb.left, curr_speed, time_ms);
		drvb.right = update_motor_at_speed(drvb.right, curr_speed, time_ms);
		traveled_dist = abs(ticks_to_dist(drvb.left.last_ticks-init_ticks));
		detection = wall_detection();
	}
	drvb = update_pos(drvb, traveled_dist, drvb.orientation);
	return zero_all(drvb);
}

struct Drivebase forward_until_detect(struct Drivebase drvb, float dist, float speed, float& traveled_dist, bool& detection)
{
	long int init_ticks = drvb.left.last_ticks;
	traveled_dist = 0;
	drvb = set_motorTime(drvb, millis());

	float curr_speed = 0.0;
	float accel_dist, decel_dist;
	accel_decel_dist(speed, kAccel, dist, accel_dist, decel_dist);

	while(!detection && traveled_dist < dist) {
		delay(kControlLoopDelay);
		long int time_ms = millis();
		
		long int diff_time_ms = time_ms - drvb.left.last_time_ms;
		float delta_s = static_cast<float>(diff_time_ms) / 1000.0f;
		curr_speed = next_speed(curr_speed, speed, kAccel, traveled_dist, accel_dist, decel_dist, delta_s);

		drvb.left = update_motor_at_speed(drvb.left, curr_speed, time_ms);
		drvb.right = update_motor_at_speed(drvb.right, curr_speed, time_ms);
		traveled_dist = abs(ticks_to_dist(drvb.left.last_ticks-init_ticks));
		detection = wall_detection();
	}

	while(curr_speed > kMinSpeed) {
		delay(kControlLoopDelay);
		long int time_ms = millis();

		long int diff_time_ms = time_ms - drvb.left.last_time_ms;
		float delta_s = static_cast<float>(diff_time_ms) / 1000.0f;
		curr_speed = next_speed(curr_speed, speed, kAccel, 5, 0, 2, delta_s);

		drvb.left = update_motor_at_speed(drvb.left, curr_speed, time_ms);
		drvb.right = update_motor_at_speed(drvb.right, curr_speed, time_ms);
	}

	drvb = update_pos(drvb, traveled_dist, drvb.orientation);
	return zero_all(drvb);
}
struct Drivebase turn_right(struct Drivebase drvb, int n_times)
{
	float dist_to_travel = PI * kRobotWidth / 4.0 * (float)n_times;
	long int init_ticks = drvb.right.last_ticks;
	drvb = set_motorTime(drvb, millis());

	float accel_dist, decel_dist;
	accel_decel_dist(kTurnSpeed, kAccel, dist_to_travel, accel_dist, decel_dist);
	float curr_speed = 0.0;
	float traveled_dist = 0.0;

	while(traveled_dist < dist_to_travel) {
		delay(kControlLoopDelay);

		long int time_ms = millis();

		long int diff_time_ms = time_ms - drvb.left.last_time_ms;
		float delta_s = static_cast<float>(diff_time_ms) / 1000.0f;
		curr_speed = next_speed(curr_speed, kTurnSpeed, kAccel, traveled_dist, accel_dist, decel_dist, delta_s);

		drvb.left = update_motor_at_speed(drvb.left, curr_speed, time_ms);
		drvb.right = update_motor_at_speed(drvb.right, -curr_speed, time_ms);
		traveled_dist = abs(ticks_to_dist(drvb.right.last_ticks-init_ticks));
	}		

	for(int i = 0; i < n_times; ++i)
		drvb = update_orientation(drvb, RIGHT);
	return zero_all(drvb);
}
struct Drivebase turn_left(struct Drivebase drvb, int n_times)
{
	float dist_to_travel = PI * kRobotWidth / 4.0 * (float)n_times;
	long int init_ticks = drvb.right.last_ticks;
	drvb = set_motorTime(drvb, millis());

	float accel_dist, decel_dist;
	accel_decel_dist(kTurnSpeed, kAccel, dist_to_travel, accel_dist, decel_dist);
	float curr_speed = 0.0;
	float traveled_dist = 0.0;

	while(traveled_dist < dist_to_travel) {
		delay(kControlLoopDelay);

		long int time_ms = millis();

		long int diff_time_ms = time_ms - drvb.left.last_time_ms;
		float delta_s = static_cast<float>(diff_time_ms) / 1000.0f;
		curr_speed = next_speed(curr_speed, kTurnSpeed, kAccel, traveled_dist, accel_dist, decel_dist, delta_s);

		drvb.left = update_motor_at_speed(drvb.left, -curr_speed, time_ms);
		drvb.right = update_motor_at_speed(drvb.right, curr_speed, time_ms);
		traveled_dist = abs(ticks_to_dist(drvb.right.last_ticks-init_ticks));
	}		

	for(int i = 0; i < n_times; ++i)
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

struct Drivebase move_to_square(struct Drivebase drvb, int direction, int n_squares)
{
	drvb = orient_toward_direction(drvb, direction);
	delay(kDecelerationDelay);
	drvb = forward_dist(drvb, kSquareSize * n_squares, kForwardSpeed);

	return drvb;
}
struct Drivebase move_to_square_or_detect(struct Drivebase drvb, int direction, int n_squares, bool& detection)
{
	drvb = orient_toward_direction(drvb, direction);
	delay(kDecelerationDelay);
	float traveled_dist;
	detection = false;
	drvb = forward_until_detect(drvb, kSquareSize, kDetectSpeed, traveled_dist, detection);

	if(detection) { // There was a wall
		delay(kDecelerationDelay);
		
		drvb = forward_dist(drvb, traveled_dist, -kForwardSpeed);
	} else {
		drvb = forward_dist(drvb, kSquareSize-traveled_dist, kForwardSpeed);
	}
	return drvb;
}
struct Drivebase orient_toward_direction(struct Drivebase drvb, int direction)
{
	if(drvb.orientation == opposite_move(direction)) {
		drvb = turn_left(drvb, 2);
		return drvb;
	} else if(drvb.orientation == direction) {
		return drvb;
	} else if(is_fastest_left(drvb.orientation, direction)) {
		drvb = turn_left(drvb);
	} else {
		drvb = turn_right(drvb);
	}
	return drvb;
}

struct Drivebase zero_all(struct Drivebase drvb)
{
	MOTOR_SetSpeed(LEFT, 0.0);
	MOTOR_SetSpeed(RIGHT, 0.0);

	drvb.left.speed = 0.0;
	drvb.right.speed = 0.0;
	return drvb;
}
struct Drivebase set_motorTime(struct Drivebase drvb, long int time_ms)
{
	drvb.left.last_time_ms = time_ms;
	drvb.right.last_time_ms = time_ms;
	return drvb;
}
