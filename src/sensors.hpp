#ifndef P28_SENSORS_HPP_
#define P28_SENSORS_HPP_

namespace p28 {

#include "Utils/Vec.hpp"

// #define PIN_IRSENSOR  to be defined when connected on the  robot
// #define PIN_COLORDETECTOR 0 to be defined when connected on the  robot
//#define PIN_LINEDETECTOR to be defined when connected on the robot

struct SensorState{ 
    unsigned long millis; // time in milliseconds
    p28::mt::i32Vec2 encoders_ticks; //number of ticks of right & left encoders:vector (left = 0,right = 1)
    p28::mt::boolVec2 bumpersState; // State of the right and left bumper (left,right)
    int IRSensor; // TO BE DEFINED
    bool proximityDetector; // value of 1 or 0
    int lineDetector; //TO BE DEFINED DEPENDING HOW IT WORKS AND ITS RESPONSE
    int colorDetector; //TO BE DEFINED DEPENDING OF ITS RESPONSE
    
};

SensorState get_sensors();

} // !p28

#endif


