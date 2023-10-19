#include <Adafruit_TCS34725.h>
#include <Wire.h>
#include <ColorSensor.hpp>

namespace p28 {

static Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
static String color_string[] = {"RED", "GREEN", "BLUE", "YELLOW", "BLACK", "WHITE"};
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

void init_color_sensor() {
    if (tcs.begin()) {
    } else {
        while (1);
    }
}

RGB get_rgb() 
{
    uint16_t clear, red, green, blue;
    tcs.getRawData(&red, &green, &blue, &clear);
    uint32_t sum = clear;
    float r, g, b;
    RGB rgb;
    r = red; r /= sum;
    g = green; g /= sum;
    b = blue; b /= sum;
    r *= 256; g *= 256; b *= 256;
    rgb.red = r;
    rgb.green = g;
    rgb.blue = b;

    return rgb;
}

bool equal_color(float col1, float col2, float treshold)
{
    if(col1*(1-treshold)<= col2 && col1*(1+treshold)>=col2)
    {
        return true;
    }
    else
    {
        return false;
    }
}

COLOR get_color()
{
    RGB rgb = get_rgb();
    if(equal_color(rgb.red, rgb.blue, 0.2) && equal_color(rgb.red, rgb.green, 0.2))
    {
        if(rgb.red >= 78)
        {
            return COLOR::WHITE;
        }
        else
        {
            return COLOR::BLACK;
        }
    }
    else
    {
        if(equal_color(rgb.red, rgb.green, 0.4))
        {
            return COLOR::YELLOW;
        }
        else if(rgb.red > rgb.green && rgb.red > rgb.blue)
        {
            return COLOR::RED;
        }
        else if(rgb.blue > rgb.green)
        {
            return COLOR::BLUE;
        }
        else
        {
            return COLOR::GREEN;
        }
    }
}

String toString(COLOR color)
{
    return color_string[color];
}