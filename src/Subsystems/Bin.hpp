#ifndef P28_BIN_HPP
#define P28_BIN_HPP

#include "HardwareState.hpp"

namespace p28 {

struct Bin
{
    //Servo motor of the color selected in the bin
     int color_selected;

    int set_bin_color(int bin_selected_color);
   
    int nb_of_blocks [3] { 0, 0, 0 };

    bool is_full();

    bool open_trap;

   HardwareState Aggregate_hardwareState (HardwareState hardwareSate);

};

}

#endif