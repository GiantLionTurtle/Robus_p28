#include "Controller.hpp"
#include <Adafruit_NeoPixel.h> 

#define LED_PIN 10 

#define LED_COUNT 11 

#define TOTAL_LED_COUNT 14 


static Adafruit_NeoPixel strip(TOTAL_LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800); 


namespace p28 {
    void innitStrip(){
        strip.begin();
        for(int i = LED_COUNT; i < TOTAL_LED_COUNT; i++){
            strip.setPixelColor(i, strip.Color(255, 255, 255));
            strip.show();
        }
    }

    void LEDOn(int r, int g, int b){
        for(int i = 0; i < LED_COUNT; i++){
            strip.setPixelColor(i, strip.Color(r, g, b));
            strip.show();
        }
    }

    void OpenLED(int color){
        int r, g, b;
        switch(color)
        {
            case kRed:
                r = 50;
                g = 0;
                b = 0;
                break;
            case KGreen:
                r = 0;
                g = 0;
                b = 50;
                break;
            case kBlue:
                r = 0;
                g = 50;
                b = 0;
                break;
            case kYellow:
                r = 50;
                g = 0;
                b = 50;
                break;
            case kAllColors:
                r = 100;
                g = 100;
                b = 100;
                break;
            default:
                r = 100;
                g = 100;
                b = 100;
        }
        LEDOn(r, g, b);
    }
}