
#include <Arduino.h>
#include <LibRobus.h>
#include <Constants.hpp>

void setup() 
{
    Serial.begin(9600);
}

void loop() 
{
printf("Mon nom est vincent");
    Serial.print("Vincent");
}

void monNom()
{
    printf("Max");
}
