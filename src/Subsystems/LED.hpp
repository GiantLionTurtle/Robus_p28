#ifndef P28_LED_HPP_
#define P28_LED_HPP_

#include "Constants.hpp"
#include "Controller.hpp"

namespace p28{
    void innitStrip();
    void LEDOn(int r, int g, int b);
    void OpenLED (int color);
}

#endif