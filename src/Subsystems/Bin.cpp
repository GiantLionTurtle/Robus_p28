#include "Bin.hpp"
#include "HardwareState.hpp"
#include "Constants.hpp"
#include "HardwareState.hpp"

namespace p28 {

void Bin::set_bin_color(int bin_selected_color){
    nb_of_blocks [bin_selected_color]++;
}

bool Bin::is_full(){
    for(int i = 0; i < 3; i++){
        if(nb_of_blocks [i] > 5){
            return true;
        }
    }
    return false;
}

HardwareState Bin::Aggregate_hardwareState(HardwareState hardwareSate){
    hardwareSate.bin_select_angles [color_selected] = kcolor_selected_angle_open;
    if(open_trap = true){
        hardwareSate.trapAngle = kopen_trap_angle;
    }
   
    return hardwareSate;
    
}


}
 
 




