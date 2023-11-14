#include "Controller.hpp"
#include "Constants.hpp"

 String get_controller_color(HardwareSerial Serial1){
    char caractere;
    String reponse;
    while(Serial1.available())
    {
        caractere = Serial1.read();
        reponse += (String)(caractere);
    }
    if(reponse!="")
    {
        String controller_color = reponse;
    }
    return reponse;
 }

