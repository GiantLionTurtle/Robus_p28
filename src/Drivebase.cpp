#include "Drivebase.hpp"
#include "Field.hpp"
#include <LibRobus.h>

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
 * @param direction Direction in which the robot travelled

 * @param x
 * x position of the robot on a 2D plane 
 * @param y 
 * y position of the robot on a 2D plane
 */
void update_pos(float dist, int direction, float& x, float& y)
{
	switch(direction)
	{
		case FRONT:
			y+=dist;
			break;
		case REAR:
			y-=dist;
			break;
		case LEFT:
			x-=dist;
			break;
		case RIGHT:
			x+=dist;
			break;
		default:
			break;
	}
}

void update_orientation(int move, int& direction)
{
	if(move == LEFT) {
		switch(direction)
		{
			case FRONT:
				direction = LEFT;
				break;
			case REAR:
				direction = RIGHT;
				break;
			case LEFT:
				direction = REAR;
				break;
			case RIGHT:
				direction = FRONT;
				break;
		}
	} else if(move == RIGHT) {
		switch(direction)
		{
			case FRONT:
				direction = RIGHT;
				break;
			case REAR:
				direction = LEFT;
				break;
			case LEFT:
				direction = FRONT;
				break;
			case RIGHT:
				direction = REAR;
				break;
		}
	}
}

Drivebase forward_dist(Drivebase drvb, float dist, float speed)
{
	return drvb;
}
Drivebase forward_until_detect(Drivebase drvb, float dist, float speed, bool& detection)
{
	return drvb;
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

	update_orientation(RIGHT, drvb.direction);
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

	update_orientation(LEFT, drvb.direction);
	return zero_all(drvb);
}

Drivebase move_to_square(Drivebase drvb, int square_x, int square_y)
{
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