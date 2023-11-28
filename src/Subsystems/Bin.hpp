#ifndef P28_BIN_HPP
#define P28_BIN_HPP

#include "HardwareState.hpp"

namespace p28 {

struct Bin
{
    //Servo motor of the color selected in the bin
     int color_selected { 0 };

   
    int nb_of_blocks [4] { 0, 0, 0, 0 };
    bool open_trap { false };

    bool is_full();
    void set_bin_color(int bin_selected_color);
    void add_block();
    void release();
    void close();

   HardwareState aggregate (HardwareState hardwareSate);

};

}

#endif