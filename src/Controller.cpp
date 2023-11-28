#include "Controller.hpp"
#include "Constants.hpp"

namespace p28 {

int get_controller_color()
{
    char character;
    String reponse;
    int color_selected = -1;
    while(Serial1.available()) {
        character = Serial1.read();

        switch(character) {
        case 'R':
            color_selected = kRed;
            break;
        case 'G':
            color_selected = KGreen;
            break;
        case 'B':
            color_selected = kBlue;
            break;
        case 'Y':
            color_selected = kYellow;
            break;
        case 'A':
            color_selected = kAllColors;
            break;
        default:
            break;
        }
    }

    return color_selected;
}

} // !p28