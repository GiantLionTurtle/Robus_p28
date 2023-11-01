
#include <LibRobus.h>
#include "sensors.hpp"
#include <Arduino.h>
#include "ProximityDetector.hpp"
#include "ColorSensor.hpp"
#include "Constants.hpp"
#include "IRSensor.hpp"
#include "LineDetector.hpp"


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
	for(int i = 0; i < 8; ++i) {
		Serial.print(is_active(state.lineDetector, i));
	}
	Serial.print(" | Proximity detector:");
	Serial.println(state.proximityDetector);
}

SensorState get_sensors()
{
	SensorState newSensorState;

	newSensorState.frontIR_dist = get_distance_ir(0);
	newSensorState.backIR_dist = get_distance_ir(1);
	newSensorState.proximityDetector = wall_detection();
	newSensorState.colorDetector = get_color();
	newSensorState.encoders_ticks = { ENCODER_Read(LEFT), ENCODER_Read(RIGHT) };
	newSensorState.bumpersState = { ROBUS_IsBumper(LEFT), ROBUS_IsBumper(RIGHT) };
	//newSensorState.lineDetector = get_ir_line();
	return newSensorState;
}

} // !p28

