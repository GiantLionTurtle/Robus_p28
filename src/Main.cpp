
#include <Arduino.h>

#include "Missions/Mission_wallRebound.hpp"
#include "Robot.hpp"
#include <Hardware/HardwareAccess.hpp>

auto mission = p28::Mission::WallRebound{};
p28::Robot robot;

void setup() 
{
	p28::init();
	mission.init();
	Serial.begin(9600);
	delay(1000);
	robot.Ziegler_Nichols_ultimateGain_drivebase(5, 4000);
}

void loop() 
{
	// // Update the robot to a snapshot of reality
	// robot = robot.snapshot();

	// // If time is up to get to the next step in the mission
	// if(robot.target.time <= robot.time_ms() || p28::Mission::close_enough(robot.current, robot.target, mission.eps))
	// 	p28::tie(robot.target, mission) = mission.update(robot);

	// // The mission gives a certain amount of time 
	// // to do a task, try to get to the desired state
	// // in time, otherwhise the mission will have
	// // to handle it
	
	// // Compute action and update the robot
	// // which changes because drivers account
	// // for their error
	// p28::ActionState actions;
	// tie(actions, robot) = robot.next_action();

	// // Do actions (move, set del, etc)
	// p28::writeActions(actions);

	// p28::SensorState sensState = p28::readSensors();
	// sensState.print();

	delay(500);
}