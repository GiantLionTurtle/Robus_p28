
#include <LibRobus.h>
#include "Utils/Vec.hpp"
#include "sensors.hpp"
#include <Arduino.h>
#include "ProximityDetector.hpp"


struct SensorState get_sensors(){
    SensorState newSensorState;
    newSensorState.millis = millis();
    newSensorState.encoders_ticks ={ENCODER_Read(LEFT),ENCODER_Read(RIGHT)};
    newSensorState.bumpersState = {ROBUS_IsBumper(0),ROBUS_IsBumper(1)};
    //newSensorState.IRSensor = {PIN_IRSENSOR};
    newSensorState.proximityDetector = wall_detection();
    //newSensorState.colorDetector = color detection function;
}

