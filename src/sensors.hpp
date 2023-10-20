#ifndef P28_SENSORS_HPP_
#define P28_SENSORS_HPP_

#include "Utils/Vec.hpp"
#include "Constants.hpp"

/*
    How SensorsState should work

    It is meant as a full representation of the robot 

    1. It is constructed using all the semi-raw data from the various sensors
    2. It puts that in a more digestible fashion such as more useful types (COLOR, Vec2)
*/


namespace p28 {

// #define PIN_IRSENSOR  to be defined when connected on the  robot
// #define PIN_COLORDETECTOR 0 to be defined when connected on the  robot
//#define PIN_LINEDETECTOR to be defined when connected on the robot

struct SensorState { 
    p28::mt::i32Vec2 encoders_ticks; //number of ticks of right & left encoders:vector (left = 0,right = 1)
    p28::mt::boolVec2 bumpersState; // State of the right and left bumper (left,right)
    int IRSensor; // &&Figureout&&
    bool proximityDetector; // value of 1 or 0
    int lineDetector; // &&Figureout&& TO BE DEFINED DEPENDING HOW IT WORKS AND ITS RESPONSE
    p28::COLOR colorDetector; // assign a color depending on the color detector response
    
};

SensorState get_sensors();

} // !p28

#endif


