#include "Controller.hpp"
#include "Constants.hpp"

 int get_controller_color(HardwareSerial Serial1){
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
        color_selected = 0;
    }
    else if(reponse == "G")
    {
        color_selected = 1;
    }
    else if(reponse == "B")
    {
       color_selected = 2;
    }
    else if(reponse == "Y")
    {
       color_selected = 3;
    }
    else if(reponse == "A")
    {
        color_selected = 4;
    }
    else 
    {
        color_selected = -1;
    }
    return color_selected;
 }

