
#ifndef P28_WHISTLEDETECTOR_HPP_
#define P28_WHISTLEDETECTOR_HPP_

#include <Arduino.h>
#include "LibRobus.h"
#include "Constants.hpp"

#define WHISTLE_PIN 18


void init_whistle();

bool whistle_detection();


#endif