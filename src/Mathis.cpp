
#include <Arduino.h>
#include "LibRobus.h"
#include "Constants.hpp"


/*void main_mathis(){
    /*SERVO_Enable(1);
    delay(100);
     int pos;
    for(pos=145; pos<180; pos++){
        SERVO_SetAngle(1, pos);
        delay(15);
    }
    delay(3000);
    for(pos=180; pos>0; pos--){
        SERVO_SetAngle(1, pos);
        delay(15);
    }
    SERVO_Disable(1);
    
   float distance;
   while (true){ 
    int retour = ROBUS_ReadIR(0);
    //Serial.println(retour);
    distance = ((-.0242*retour)+23.646);
   Serial.println(distance);
   delay(1000);
   }

}
*/

