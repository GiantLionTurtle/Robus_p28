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
	// float frontIR_dist;
	// float backIR_dist;

	// bool proximityDetector; // value of 1 or 0
	char lineDetector; // &&Figureout&& TO BE DEFINED DEPENDING HOW IT WORKS AND ITS RESPONSE
	// COLOR colorDetector; // assign a color depending on the color detector response

	mt::i32Vec2 block_offset;
	bool block_in_claw { false };

	static void init();
};

SensorState get_sensors();
void print (SensorState state);
} // !p28


#endif


