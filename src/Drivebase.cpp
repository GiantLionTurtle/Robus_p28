
#include "Drivebase.hpp"

namespace p28 {

float ticks_to_dist(uint32_t ticks)
{
	return static_cast<float>(ticks) / 3200.0 * TWO_PI * p28::kWheelRadius;
}

} // !p28