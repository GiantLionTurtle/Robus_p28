
#ifndef P28_CAMERA_HPP_
#define P28_CAMERA_HPP_

#include <Utils/Vec2.hpp>
#include <Utils/Pair.hpp>
#include <Pixy2.h>
#include <Pixy2I2C.h>
#include <CompileFlags.hpp>

namespace p28 {

struct Camera {
#ifdef ENABLE_CAMERA
	Pixy2I2C pixy;
#endif

	void init();

	// Returns the centroid offset to the alignement line of the claw
	// + if the whole block is within the claw bounding box
	// of the biggest block of a certain color
	void blockOffset(int targcolor, mt::Vec2& offset, bool& in_claw, int& color);

	// Converts tracking signature to internal color code
	int signature_to_color(int sig);
};

} // !p28

#endif