
#include "HardwareState.hpp"

namespace p28 {

struct Bin
{
    //Servo motor of the color selected in the bin
     int color_selected;

    int set_bin_color(int bin_selected_color);
   
    int nb_of_blocks [3];

    bool is_full();

   HardwareState Aggregate_hardwareState (HardwareState hardwareSate);

};

}