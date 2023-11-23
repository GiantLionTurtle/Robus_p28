#include "Controller.hpp"
#include "Constants.hpp"

namespace p28 {

 int get_controller_color()
 {
    char caractere;
    String reponse;
    int color_selected;
    while(Serial1.available())
    {
        caractere = Serial1.read();
        reponse += (String)(caractere);
    }
    if(reponse == "R")
    {
        color_selected = kRed;
    }
    else if(reponse == "G")
    {
        color_selected = KGreen;
    }
    else if(reponse == "B")
    {
       color_selected = kBlue;
    }
    else if(reponse == "Y")
    {
       color_selected = kYellow;
    }
    else if(reponse == "A")
    {
        color_selected = KAll;
    }
    else 
    {
        color_selected = -1;
    }
    return color_selected;
 }

}