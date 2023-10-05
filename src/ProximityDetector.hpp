
#ifndef P28_PROXIMITYDETECTOR_HPP_
#define P28_PROXIMITYDETECTOR_HPP_

#include <Arduino.h>
#include "LibRobus.h"
#include "Constants.hpp"

#define GREEN_PIN 15
#define RED_PIN 14



void init_detector();

bool wall_detection();


#endif