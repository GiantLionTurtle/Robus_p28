
#include <LibRobus.h>
#include "SensorsState.hpp"
#include <Arduino.h>
#include "Constants.hpp"
#include "Sensors/LineDetector.hpp"
#include "Sensors/Camera.hpp"

namespace p28 {

static Camera camera;

void SensorState::init()
{
	camera.init();
}

void print(SensorState state)
{
	Serial.print("bumper:");
	print(state.bumpersState);
	
	Serial.print(" | encoders:");
	print(state.encoders_ticks);
	
	Serial.print(" | Line detector:");
	for(int i = 0; i < 8; ++i) {
		Serial.print(is_active(state.lineDetector, i));
	}
	Serial.print(" | Block offset: ");
	print(state.block_offset);
	Serial.println();
}

SensorState get_sensors(SensorState prevSensState, int targetColor)
{
	SensorState newSensorState;

	newSensorState.lineDetector = get_ir_line();
	newSensorState.bumpersState = { ROBUS_IsBumper(LEFT), ROBUS_IsBumper(RIGHT) };

	newSensorState.block_color = prevSensState.block_color;
	newSensorState.block_in_claw = prevSensState.block_in_claw;
	newSensorState.block_offset = prevSensState.block_offset;
	camera.blockOffset(targetColor, newSensorState.block_offset, newSensorState.block_in_claw, newSensorState.block_color);

	// Encoders must be placed last to minimize unacounted for delays
	newSensorState.encoders_ticks = { ENCODER_Read(LEFT), ENCODER_Read(RIGHT) };
	return newSensorState;
}


} // !p28

