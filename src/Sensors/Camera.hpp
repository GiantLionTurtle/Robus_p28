
#ifndef P28_CAMERA_HPP_
#define P28_CAMERA_HPP_

#include <Utils/Vec2.hpp>
#include <Pixy2.h>

namespace p28 {

struct Camera {
	Pixy2 pixy;

	void init();

	// Returns the centroid offset to the claw centroid
	// of the biggest block of a certain color
	mt::i32Vec2 blockOffset(int color);

	int signature_to_color(int sig);
};

} // !p28

#endif