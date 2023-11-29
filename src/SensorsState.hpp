#ifndef P28_SENSORS_HPP_
#define P28_SENSORS_HPP_

#include "Utils/Vec2.hpp"
#include "Constants.hpp"

/*
	How SensorsState should work

	It is meant as a full representation of the robot 

	1. It is constructed using all the semi-raw data from the various sensors
	2. It puts that in a more digestible fashion such as more useful types (COLOR, Vec2)
*/


namespace p28 {

struct SensorState {
	mt::i32Vec2 encoders_ticks; //number of ticks of right & left encoders:vector (left = 0,right = 1)
	mt::boolVec2 bumpersState; // State of the right and left bumper (left,right)

	char lineDetector; // &&Figureout&& TO BE DEFINED DEPENDING HOW IT WORKS AND ITS RESPONSE

	mt::Vec2 block_offset { 0.0 };
	int block_color { -1 };
	bool block_in_claw { false };

	static void init();
};

SensorState get_sensors(SensorState prevSensState, int targetColor);
void print(SensorState state);
} // !p28


#endif


