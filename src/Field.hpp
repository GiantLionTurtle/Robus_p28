
#ifndef P28_FIELD_HPP_
#define P28_FIELD_HPP_

#include "Utils/Geometry.hpp"

namespace p28 {

namespace Field {

const float Foot = 0.3048;

const mt::Vec2 kDimensions { 2.0, 2.0 };

const float kDumpBorderOffset = 0.15;

const mt::Vec2 kDumps[4] {
	{ kDumpBorderOffset, kDumpBorderOffset }, // Red
	{ kDimensions.x-kDumpBorderOffset, kDumpBorderOffset }, // Green
	{ kDimensions.x-kDumpBorderOffset, kDimensions.y-kDumpBorderOffset }, // Blue
	{ kDumpBorderOffset, kDimensions.y-kDumpBorderOffset } // Yellow
};

const mt::Vec2 kDumpHeading[4]{
	{1, -1}, // Red
	{1, 1}, // Green
	{-1, 1}, // Blue
	{-1, -1} // Yellow
};

const mt::Vec2 kDumpConnectors[4] {
	{ kDumpBorderOffset, kDimensions.y/2 },
	{ kDimensions.x/2, kDumpBorderOffset },
	{ kDimensions.x-kDumpBorderOffset, kDimensions.y/2 },
	{ kDimensions.x/2, kDimensions.y-kDumpBorderOffset}
};

} // !Field

} // !p28

#endif