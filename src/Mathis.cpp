
#include <Arduino.h>
#include "LibRobus.h"
#include "Constants.hpp"

void buzzerFin (void);

void setup (){
    BoardInit();
    buzzerFin();
}
void loop (){

}
void buzzerFin (void) {
    AX_BuzzerON (440,120);
}
