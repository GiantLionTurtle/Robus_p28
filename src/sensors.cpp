
#include <LibRobus.h>
#include "sensors.hpp"
#include <Arduino.h>
#include "ProximityDetector.hpp"
#include "ColorSensor.hpp"
#include "Constants.hpp"

namespace p28 {

SensorState get_sensors()
{
    SensorState newSensorState;
    newSensorState.encoders_ticks = { ENCODER_Read(LEFT), ENCODER_Read(RIGHT) };
    newSensorState.bumpersState = { ROBUS_IsBumper(LEFT), ROBUS_IsBumper(RIGHT) };
    // newSensorState.IRSensor = {PIN_IRSENSOR};
    newSensorState.frontIR_dist = {};
    newSensorState.proximityDetector = wall_detection();
    newSensorState.colorDetector = get_color();
    //newSensorState.lineDetector = lineDetector functions
    return newSensorState;
}

} // !p28

