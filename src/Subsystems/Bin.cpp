#include "Bin.hpp"
#include "HardwareState.hpp"
#include "Constants.hpp"
#include "HardwareState.hpp"

namespace p28 {

void Bin::set_bin_color(int bin_selected_color)
{
    color_selected = bin_selected_color;
}
void Bin::add_block()
{
    nb_of_blocks [color_selected]++;
}
void Bin::release()
{
    open_trap = true;
    for(int i = 0; i < 4; ++i) {
        nb_of_blocks[i] = 0;
    }
}
void Bin::close()
{
    Serial.println("Close trap!");
    open_trap = false;
}
bool Bin::is_full()
{
    for(int i = 0; i < 4; i++){
        if(nb_of_blocks [i] > kBinCapacity){
            return true;
        }
    }
    return false;
}

HardwareState Bin::aggregate(HardwareState hardwareSate)
{
    if(color_selected < 3) {
        hardwareSate.bin_select_angles [color_selected] = kcolor_selected_angle_open;
    }
    if(open_trap){
        hardwareSate.trapAngle = kopen_trap_angle;
    }
   
    return hardwareSate;
}


}
 
 




