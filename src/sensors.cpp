
#include <LibRobus.h>
#include "sensors.hpp"
#include <Arduino.h>
#include "ProximityDetector.hpp"
#include "ColorSensor.hpp"
#include "Constants.hpp"
#include "IRSensor.hpp"


namespace p28 {

void printSensor(SensorState state)
{
    Serial.println("bumper:");
    print(state.bumpersState);
    Serial.println();
    Serial.println("Color detector:");
    Serial.println(toString(state.colorDetector));
    Serial.println("encoders:");
    print(state.encoders_ticks);
    Serial.println();
    Serial.println("FrontIR:");
    Serial.println(state.frontIR_dist);
    Serial.println("Line detector:");
    Serial.println(state.lineDetector);
    Serial.println("Proximity detector:");
    Serial.println(state.proximityDetector);
}

SensorState get_sensors()
{
    SensorState newSensorState;
    newSensorState.encoders_ticks = { ENCODER_Read(LEFT), ENCODER_Read(RIGHT) };
    newSensorState.bumpersState = { ROBUS_IsBumper(LEFT), ROBUS_IsBumper(RIGHT) };
    // newSensorState.IRSensor = {PIN_IRSENSOR};

    //index 0 --> adjust index depending which IR captor we want, create const for each IR
    newSensorState.frontIR_dist = get_distance_ir(0);  
    newSensorState.proximityDetector = wall_detection();
    newSensorState.colorDetector = get_color();
    //newSensorState.lineDetector = lineDetector functions
    return newSensorState;
}

} // !p28

