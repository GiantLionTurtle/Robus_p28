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
 * 0: forwards
 * 1: backwards
 * 2: left
 * 3: right
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

float ticks_to_dist(uint32_t ticks)
{
	return static_cast<float>(ticks) / 3200.0 * TWO_PI * p28::kWheelRadius;
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
 * 0: forwards
 * 1: backwards
 * 2: left
 * 3: right
 * @param x
 * x position of the robot on a 2D plane 
 * @param y 
 * y position of the robot on a 2D plane
 */
void update_pos(float dist, int direction, float& x, float& y)
{
	switch(direction)
	{
		case 0:
			y+=dist;
			break;
		case 1:
			y-=dist;
			break;
		case 2:
			x-=dist;
			break;
		case 3:
			x+=dist;
			break;
		default:
			break;
	}
}

void update_orientation(int move, int& direction)
{
	switch(move)
	{
case 0:
			break;
		case 1:
			direction = abs(direction-move);
			break;
		case 2:
			switch(direction)
			{
				case 0:
					direction = 2;
					break;
				case 1:
					direction = 3;
					break;
				case 2:
					direction = 1;
					break;
				case 3:
					direction = 0;
					break;
			}
		case 3:
			switch(direction)
			{
				case 0:
					direction = 3;
					break;
				case 1:
					direction = 2;
					break;
				case 2:
					direction = 0;
					break;
				case 3:
					direction = 1;
					break;
			}
		
	}
}

Drivebase forward_dist(Drivebase drvb, float dist, float speed)
{
	long int init_ticks = drvb.left.last_ticks;
	float distance_parcourue = 0;
	while(distance_parcourue < dist)
	{
		long int time_ms = millis();
		drvb.left = update_motor_at_speed(drvb.left, speed, time_ms);
		drvb.right = update_motor_at_speed(drvb.right, speed, time_ms);
		distance_parcourue = ticks_to_dist(drvb.left.last_ticks-init_ticks);
	}
	drvb.left = update_motor_at_speed(drvb.left, 0, millis());
	drvb.right = update_motor_at_speed(drvb.right, 0, millis());
	update_pos(distance_parcourue, drvb.direction, drvb.x, drvb.y);
	return drvb;
}
Drivebase forward_until_detect(Drivebase drvb, float dist, float speed, bool& detection)
{
	long int init_ticks = drvb.left.last_ticks;
	float distance_parcourue = 0;
	while(!detection)
	{
		long int time_ms = millis();
		drvb.left = update_motor_at_speed(drvb.left, speed, time_ms);
		drvb.right = update_motor_at_speed(drvb.right, speed, time_ms);
		distance_parcourue = ticks_to_dist(drvb.left.last_ticks-init_ticks);
	}
	drvb.left = update_motor_at_speed(drvb.left, 0, millis());
	drvb.right = update_motor_at_speed(drvb.right, 0, millis());
	update_pos(distance_parcourue, drvb.direction, drvb.x, drvb.y);
	return drvb;
}
Drivebase turn_right(Drivebase drvb)
{

update_motor_at_speed(drvb.left, 0.5,(((TWO_PI*kWheelRadius)/4)*(0.5*kMaxVel))*1000);
	update_motor_at_speed(drvb.right,-0.5,(((TWO_PI*kWheelRadius)/4)*(0.5*kMaxVel))*1000);
	update_orientation(3,drvb.direction);
	return drvb;
}
Drivebase turn_left(Drivebase drvb)
{
update_motor_at_speed(drvb.left, -0.5,(((TWO_PI*kWheelRadius)/4)*(0.5*kMaxVel))*1000);
	update_motor_at_speed(drvb.right,0.5,(((TWO_PI*kWheelRadius)/4)*(0.5*kMaxVel))*1000);
	update_orientation(2,drvb.direction);
	return drvb;
}

Drivebase move_to_square(Drivebase drvb, int square_x, int square_y)
{

}


} // !p28