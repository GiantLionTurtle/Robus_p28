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
void update_pos(double dist, int direction, double& x, double& y);
// move; LEFT or RIGHT, changes the robot direction
// direction is !!! RELATIVE TO THE FIELD !!! 
void update_orientation(int move, int& direction);


// Public functions

double ticks_to_dist(int32_t ticks)
{
	return static_cast<float>(ticks) / 3200 * TWO_PI * kWheelRadius;
}
double comp_accel_dist(double accel, double currSpeed, double targSpeed)
{
	return (targSpeed - currSpeed) / accel / 2.0;
}

struct Motor get_motor_speed(struct Motor motor, double delta_s)
{
	int32_t current_ticks = ENCODER_Read(motor.ID);
	int32_t ticks_diff = current_ticks - motor.last_ticks;
	motor.speed = ticks_to_dist(ticks_diff) / delta_s;
	motor.last_ticks = current_ticks;
	return motor;
}
struct Motor update_motor_at_speed(struct Motor motor, double set_speed, long int time_ms)
{
	long int diff_time_ms = time_ms - motor.last_time_ms;
	double delta_s = static_cast<float>(diff_time_ms) / 1000.0f;

	motor.last_time_ms = time_ms;
	motor = get_motor_speed(motor, delta_s);
	motor.error = update_error(motor.error, motor.speed, set_speed, delta_s);
	double harware_set = get(motor.pid, motor.error);

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
struct Drivebase update_pos(struct Drivebase drvb, double dist, int direction)
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

double do_accell(double curr, double accel, double delta_s)
{
	return curr + accel * delta_s;
}
double next_speed(double curr_speed, double target, double accel, double curr_dist, double accel_dist, double decel_dist, double delta_s)
{
	if(curr_dist < accel_dist) {
		// // Serial.println("accel");
		return min(do_accell(curr_speed, accel, delta_s), target);
	} else if(curr_dist > decel_dist) {
		// // Serial.println("decl");
		return max(do_accell(curr_speed, -accel, delta_s), kMinSpeed);
	} else {
		// // Serial.println("Stable");
		return target;
	}

}
void accel_decel_dist(double target_speed, double accel, double max_dist, double& accel_dist, double& decel_dist)
{
	// accel_dist = abs(comp_accel_dist(accel, kMinSpeed, target_speed));
	accel_dist = kAccelDist;
	if(max_dist < 2*accel_dist) {
		accel_dist = max_dist/2.0;
	}
	decel_dist = max_dist - accel_dist;
}
struct Drivebase forward_dist(struct Drivebase drvb, double dist, double speed)
{
	long int init_ticks_right = drvb.right.last_ticks;
	long int init_ticks_left = drvb.left.last_ticks;

	long int init_time_ms = millis();
	double traveled_dist = 0;
	drvb = set_motorTime(drvb, init_time_ms);

	double curr_speed = kMinSpeed;
	double accel_dist, decel_dist;
	accel_decel_dist(speed, kAccel, dist, accel_dist, decel_dist);
	double dist_mult = speed < 0 ? -1.0 : 1.0;

	while(traveled_dist < dist) {
		delay(kControlLoopDelay);

		long int time_ms = millis();

		long int diff_time_ms = time_ms - drvb.left.last_time_ms;
		double delta_s = static_cast<float>(diff_time_ms) / 1000.0f;
		curr_speed = dist_mult*next_speed(abs(curr_speed), abs(speed), kAccel, traveled_dist, accel_dist, decel_dist, delta_s);

		drvb.left = update_motor_at_speed(drvb.left, curr_speed, time_ms);
		drvb.right = update_motor_at_speed(drvb.right, curr_speed, time_ms);
		traveled_dist = ticks_to_dist((abs(drvb.left.last_ticks-init_ticks_left) + abs(drvb.right.last_ticks-init_ticks_right)) / 2);
	}

	drvb = update_pos(drvb, dist_mult*traveled_dist, drvb.orientation);
	return zero_all(drvb);
}

struct Drivebase forward_until_detect(struct Drivebase drvb, double dist, double speed, double& traveled_dist, bool& detection)
{
	long int init_ticks_right = drvb.right.last_ticks;
	long int init_ticks_left = drvb.left.last_ticks;

	traveled_dist = 0;
	drvb = set_motorTime(drvb, millis());


	double curr_speed = 0.0;
	double accel_dist, decel_dist;
	accel_decel_dist(speed, kAccel, dist, accel_dist, decel_dist);
	// // Serial.print("accel decel ");
	// // Serial.print(accel_dist);
	// // Serial.print(",  ");
	// // Serial.println(decel_dist);
	while(!detection && traveled_dist < dist) {
		delay(kControlLoopDelay);
		// // Serial.print(traveled_dist);
		// // Serial.print(",  ");
		// // Serial.println(curr_speed);
		long int time_ms = millis();
		
		long int diff_time_ms = time_ms - drvb.left.last_time_ms;
		double delta_s = static_cast<float>(diff_time_ms) / 1000.0f;
		curr_speed = next_speed(curr_speed, speed, kAccel, traveled_dist, accel_dist, decel_dist, delta_s);

		drvb.left = update_motor_at_speed(drvb.left, curr_speed, time_ms);
		drvb.right = update_motor_at_speed(drvb.right, curr_speed, time_ms);
		traveled_dist = ticks_to_dist((abs(drvb.left.last_ticks-init_ticks_left) + abs(drvb.right.last_ticks-init_ticks_right)) / 2);
		detection = wall_detection() && fmod(traveled_dist, kSquareSize) < kSquareSize * 0.5;
	}

	while(curr_speed > kMinSpeed) {
		delay(kControlLoopDelay);
		// // Serial.println("in slowdown");
		long int time_ms = millis();

		long int diff_time_ms = time_ms - drvb.left.last_time_ms;
		double delta_s = static_cast<float>(diff_time_ms) / 1000.0f;
		curr_speed = next_speed(curr_speed, speed, kCatastrophicDecel, 5, 0, 2, delta_s);

		drvb.left = update_motor_at_speed(drvb.left, curr_speed, time_ms);
		drvb.right = update_motor_at_speed(drvb.right, curr_speed, time_ms);
	}

	traveled_dist = ticks_to_dist((abs(drvb.left.last_ticks-init_ticks_left) + abs(drvb.right.last_ticks-init_ticks_right)) / 2);
	drvb = update_pos(drvb, traveled_dist, drvb.orientation);
	return zero_all(drvb);
}
struct Drivebase turn_right(struct Drivebase drvb, int n_times)
{
	double dist_to_travel = kCircumference / 4.0 * (float)n_times;// - (0.01 - (0.005*it++));
	long int init_ticks_right = drvb.right.last_ticks;
	long int init_ticks_left = drvb.left.last_ticks;
	drvb = set_motorTime(drvb, millis());

	double accel_dist, decel_dist;
	accel_decel_dist(kTurnSpeed, kTurnAccel, dist_to_travel, accel_dist, decel_dist);
	double curr_speed = kMinSpeed;
	double traveled_dist = 0.0;

	while(traveled_dist <= dist_to_travel) {
		delay(kControlLoopDelay);

		long int time_ms = millis();

		long int diff_time_ms = time_ms - drvb.left.last_time_ms;
		double delta_s = static_cast<float>(diff_time_ms) / 1000.0f;
		curr_speed = next_speed(curr_speed, kTurnSpeed, kTurnAccel, traveled_dist, accel_dist, decel_dist, delta_s);

		drvb.right = update_motor_at_speed(drvb.right, -curr_speed, time_ms);
		drvb.left = update_motor_at_speed(drvb.left, curr_speed, time_ms);

		traveled_dist = (abs(ticks_to_dist(drvb.right.last_ticks-init_ticks_right)) + 
						abs(ticks_to_dist(drvb.left.last_ticks - init_ticks_left))) / 2.0;
	}		

	for(int i = 0; i < n_times; ++i)
		drvb = update_orientation(drvb, RIGHT);

	return zero_all(drvb);
}
struct Drivebase turn_left(struct Drivebase drvb, int n_times)
{
	double dist_to_travel = kCircumference / 4.0 * (float)n_times;
	long int init_ticks_right = drvb.right.last_ticks;
	long int init_ticks_left = drvb.left.last_ticks;

	drvb = set_motorTime(drvb, millis());

	double accel_dist, decel_dist;
	accel_decel_dist(kTurnSpeed, kTurnAccel, dist_to_travel, accel_dist, decel_dist);
	double curr_speed = kMinSpeed;
	double traveled_dist = 0.0;

	while(traveled_dist <= dist_to_travel) {
		delay(kControlLoopDelay);

		long int time_ms = millis();

		long int diff_time_ms = time_ms - drvb.left.last_time_ms;
		double delta_s = static_cast<float>(diff_time_ms) / 1000.0f;
		curr_speed = next_speed(curr_speed, kTurnSpeed, kTurnAccel, traveled_dist, accel_dist, decel_dist, delta_s);

		drvb.left = update_motor_at_speed(drvb.left, -curr_speed, time_ms);
		drvb.right = update_motor_at_speed(drvb.right, curr_speed, time_ms);
		traveled_dist = (abs(ticks_to_dist(drvb.right.last_ticks-init_ticks_right)) + 
						abs(ticks_to_dist(drvb.left.last_ticks - init_ticks_left))) / 2.0;
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


struct Drivebase realign_left(struct Drivebase drvb, int diff)
{
	while(drvb.left.last_ticks < drvb.right.last_ticks + diff) {
		delay(kControlLoopDelay);
		long int time_ms = millis();

		drvb.left = update_motor_at_speed(drvb.left, kRealignSpeed, time_ms);
		drvb.right = update_motor_at_speed(drvb.right, -kRealignSpeed, time_ms);
	}
	return drvb;
}
struct Drivebase realign_right(struct Drivebase drvb, int diff)
{
	while(drvb.right.last_ticks < drvb.left.last_ticks - diff) {
		delay(kControlLoopDelay);
		long int time_ms = millis();
		
		drvb.left = update_motor_at_speed(drvb.left, -kRealignSpeed, time_ms);
		drvb.right = update_motor_at_speed(drvb.right, kRealignSpeed, time_ms);
	}
	return drvb;
}

// Tries to make the number of ticks right and left equal
struct Drivebase realign(struct Drivebase drvb, int diff)
{
	static int it = 0;
	if(abs(drvb.left.last_ticks - drvb.right.last_ticks) > abs(diff) + kTicksPerRotation) {
		return drvb;
	}

	drvb = set_motorTime(drvb, millis());

	if(it % 2 == 0) {
		drvb = realign_left(drvb, diff);
		drvb = realign_right(drvb, diff);
	} else {
		drvb = realign_right(drvb, diff);
		drvb = realign_left(drvb, diff);
	}

	it++;
	return zero_all(drvb);
}

struct Drivebase move_to_square(struct Drivebase drvb, int direction, int n_squares, bool allow_back)
{
	delay(kDecelerationDelay);

	double speed = kForwardSpeed;
	if(direction == REAR && drvb.orientation == FRONT && allow_back) {
		speed = -speed;
	} else {
		drvb = orient_toward_direction(drvb, direction);
	}
	// if(drvb.orientation == FRONT)
	// 	drvb = realign(drvb, 0);


	long int init_ticks_right = drvb.right.last_ticks;
	long int init_ticks_left = drvb.left.last_ticks;
	long int diff = init_ticks_left - init_ticks_right;


	delay(kDecelerationDelay);
	drvb = forward_dist(drvb, kSquareSize * n_squares - kInertiaDist, speed);
	drvb = realign(drvb, diff);

	return drvb;
}
struct Drivebase move_to_square_or_detect(struct Drivebase drvb, int direction, int n_squares, int& n_squares_done)
{
	int sq_x_init = drvb.sq_x;
	int sq_y_init = drvb.sq_y;

	delay(kDecelerationDelay);

	double speed = kDetectSpeed;
	// if(direction == REAR && drvb.orientation == FRONT) {
	// 	speed = -speed;
	// } else {
		drvb = orient_toward_direction(drvb, direction);
	// }
	// if(drvb.orientation == FRONT)
	// 	drvb = realign(drvb, 0);

	delay(kDecelerationDelay);

	long int init_ticks_right = drvb.right.last_ticks;
	long int init_ticks_left = drvb.left.last_ticks;
	long int diff = init_ticks_left - init_ticks_right;

	double traveled_dist;
	bool detection = false;

	double dist_to_travel = n_squares*kSquareSize;
	double x_within_square = drvb.x - drvb.sq_x * kSquareSize - kSquareSize / 2.0;
	double y_within_square = drvb.y - drvb.sq_y * kSquareSize - kSquareSize / 2.0;
	switch(direction) {
	case FRONT: 
		dist_to_travel -= y_within_square;
		break;
	case REAR:
		dist_to_travel += y_within_square;
		break;
	case RIGHT:
		dist_to_travel -= x_within_square;
		break;
	case LEFT:
		dist_to_travel += x_within_square;
		break;
	default:
		break;
	}

	drvb = forward_until_detect(drvb, dist_to_travel-kInertiaForwardDist, speed, traveled_dist, detection);
	// n_squares_done = n_squares;
	if(detection) {// && traveled_dist > 0.02) { // There was a wall
		delay(kDecelerationDelay);
		// // Serial.print("Go back ");
		// // Serial.println(traveled_dist);

		double offset_dist = max(fmod(traveled_dist, kSquareSize)-kInertiaDist, 0.0);
		// drvb = forward_dist(drvb, offset_dist-kInertiaDist, -speed);
		drvb = forward_dist(drvb, offset_dist, -speed);
		// n_squares_done = 
	}
	// if(drvb.orientation == FRONT)
	// 	drvb = realign(drvb, 0);
	drvb = realign(drvb, diff);

	n_squares_done = abs(sq_x_init - drvb.sq_x) + abs(sq_y_init - drvb.sq_y);
	return drvb;
}
struct Drivebase orient_toward_direction(struct Drivebase drvb, int direction)
{
	if(drvb.orientation == opposite_move(direction)) {
		drvb = turn_left(drvb);
		delay(kDecelerationDelay);
		drvb = turn_left(drvb);
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
