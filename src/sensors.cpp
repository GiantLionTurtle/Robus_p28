
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
	Serial.print("bumper:");
	print(state.bumpersState);
	
	Serial.print(" | Color detector:");
	Serial.print(toString(state.colorDetector));
	Serial.print(" | encoders:");
	print(state.encoders_ticks);
	
	Serial.print(" | FrontIR:");
	Serial.print(state.frontIR_dist);
	Serial.print(" | Line detector:");
	Serial.print(state.lineDetector);
	Serial.print(" | Proximity detector:");
	Serial.println(state.proximityDetector);
}

SensorState get_sensors()
{
	SensorState newSensorState;
	newSensorState.encoders_ticks = { ENCODER_Read(LEFT), ENCODER_Read(RIGHT) };
	newSensorState.bumpersState = { ROBUS_IsBumper(LEFT), ROBUS_IsBumper(RIGHT) };
	// newSensorState.IRSensor = {PIN_IRSENSOR};

	//index 0 --> adjust index depending which IR captor we want, create const for each IR
	// newSensorState.frontIR_dist = get_distance_ir(0);  
	// newSensorState.proximityDetector = wall_detection();
	// newSensorState.colorDetector = get_color();
	//newSensorState.lineDetector = lineDetector functions
	return newSensorState;
}

} // !p28

