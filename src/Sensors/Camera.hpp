
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

	// Returns the centroid offset to the claw centroid
	// of the biggest block of a certain color
	Pair<mt::i32Vec2, bool> blockOffset(int color);

	int signature_to_color(int sig);
};

} // !p28

#endif