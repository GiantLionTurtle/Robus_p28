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
