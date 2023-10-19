#ifndef P28_COLOR_SENSOR_HPP_
#define P28_COLOR_SENSOR_HPP_

#include <Constants.hpp>

namespace p28 {

struct RGB{
    float red;
    float green;
    float blue;
};

void init_color_sensor();
RGB get_rgb();
COLOR get_color();
String toString(COLOR color);
} // !p28
#endif