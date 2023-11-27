
#include <LibRobus.h>
#include "SensorsState.hpp"
#include <Arduino.h>
#include "Sensors/ProximityDetector.hpp"
#include "Sensors/ColorSensor.hpp"
#include "Constants.hpp"
#include "Sensors/IRSensor.hpp"
#include "Sensors/LineDetector.hpp"
#include "Sensors/Camera.hpp"

namespace p28 {

static Camera camera;

void SensorState::init()
{
	// init_color_sensor();
	// init_detector();
	camera.init();
}

void print(SensorState state)
{
	Serial.print("bumper:");
	print(state.bumpersState);
	
	// Serial.print(" | Color detector:");
	// Serial.print(toString(state.colorDetector));
	Serial.print(" | encoders:");
	print(state.encoders_ticks);
	
	// Serial.print(" | FrontIR:");
	// Serial.print(state.frontIR_dist);
	Serial.print(" | Line detector:");
	for(int i = 0; i < 8; ++i) {
		Serial.print(is_active(state.lineDetector, i));
	}
	// Serial.print(" | Proximity detector:");
	// Serial.println(state.proximityDetector);
	Serial.print(" | Block offset: ");
	print(state.block_offset);
	Serial.println();
}

SensorState get_sensors(int targetColor)
{
	SensorState newSensorState;

	// newSensorState.frontIR_dist = get_distance_ir(0);
	// newSensorState.backIR_dist = get_distance_ir(1);
	// newSensorState.proximityDetector = wall_detection();
	// newSensorState.colorDetector = get_color();
	newSensorState.lineDetector = get_ir_line();
	newSensorState.bumpersState = { ROBUS_IsBumper(LEFT), ROBUS_IsBumper(RIGHT) };

	tie(newSensorState.block_offset, newSensorState.block_in_claw) = camera.blockOffset(targetColor);

	// Encoders must be placed last to minimize unacounted for delays
	newSensorState.encoders_ticks = { ENCODER_Read(LEFT), ENCODER_Read(RIGHT) };
	return newSensorState;
}


} // !p28

