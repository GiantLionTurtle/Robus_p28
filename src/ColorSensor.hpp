#ifndef P28_COLOR_SENSOR_HPP_
#define P28_COLOR_SENSOR_HPP_

struct RGB{
    float red;
    float green;
    float blue;
};

enum COLOR {RED, GREEN, BLUE, YELLOW, BLACK, WHITE};

void init_color_sensor();
RGB get_rgb();
bool equal_color(float col1, float col2, float treshold);
COLOR get_color();
#endif