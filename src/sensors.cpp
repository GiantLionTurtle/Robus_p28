
#include <LibRobus.h>
#include "sensors.hpp"
#include <Arduino.h>
#include "ProximityDetector.hpp"
#include "ColorSensor.hpp"
#include "Constants.hpp"

namespace p28 {

void printSensor(SensorState state)
{
    print(state.bumpersState);
    Serial.println();
    Serial.println(toString(state.colorDetector));
    print(state.encoders_ticks);
    Serial.println();
    Serial.println(state.frontIR_dist);
    Serial.println(state.lineDetector);
    Serial.println(state.proximityDetector);
}

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

