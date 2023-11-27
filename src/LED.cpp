#include "Controller.hpp"

namespace p28 {
    void OpenLED(int controller_color){
        int color;
        color = get_controller_color();
        switch(color)
        {
            case 0:
                Serial.println("Red");
                break;
            case 1:
                Serial.println("Green");
                break;
            case 2:
                Serial.println("Blue");
                break;
            case 3:
                Serial.println("Yellow");
                break;
            case 4:
                Serial.println("All");
                break;
            default:
                Serial.println("Nothing");
        }
    }
}