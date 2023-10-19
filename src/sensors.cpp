
#include <LibRobus.h>
#include "sensors.hpp"
#include <Arduino.h>
#include "ProximityDetector.hpp"

namespace p28 {

SensorState get_sensors()
{
    SensorState newSensorState;
    newSensorState.encoders_ticks = { ENCODER_Read(LEFT), ENCODER_Read(RIGHT) };
    newSensorState.bumpersState = { ROBUS_IsBumper(LEFT), ROBUS_IsBumper(RIGHT) };
    //newSensorState.IRSensor = {PIN_IRSENSOR};
    newSensorState.proximityDetector = wall_detection();
    //newSensorState.colorDetector = color detection function;

    return newSensorState;
}

} // !p28

