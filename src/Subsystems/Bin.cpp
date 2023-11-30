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
    Serial.println("Release!! trap!");
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
        if(nb_of_blocks [i] > 8){
            return true;
        }
    }
    return false;
}

HardwareState Bin::aggregate(HardwareState hardwareSate)
{
    if(color_selected < 3) {
        for(int i = 3; i >= color_selected; --i) {
            hardwareSate.bin_select_angles [i] = kBinSelect_openAngle;
        }
    }
    if(open_trap) {
        hardwareSate.trapAngle = kBinTrap_openAngle;
    }
   
    return hardwareSate;
}


}
 
 




