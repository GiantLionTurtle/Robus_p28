
#ifndef P28_FIELD_HPP_
#define P28_FIELD_HPP_

#include "Utils/Geometry.hpp"

namespace p28 {

namespace Field {

const mt::Line zone_1_to_2_line { .origin=mt::Vec2(0.0), .dir=mt::Vec2(2.0) };
const mt::Line zone_2_to_3_line { .origin=mt::Vec2(0.0), .dir=mt::Vec2(2.0) };
const mt::Line zone_5_to_6_line { .origin=mt::Vec2(0.0), .dir=mt::Vec2(2.0) };
const mt::Line zone_8_to_9_line { .origin=mt::Vec2(0.0), .dir=mt::Vec2(2.0) };
 
constexpr int n_zones = 11;
const mt::Box zones_boxes[n_zones] {
	mt::Box{}, // 0
	mt::Box{}, // 1
	mt::Box{}, // 2
	mt::Box{}, // 3
	mt::Box{}, // 4
	mt::Box{}, // 5
	mt::Box{}, // 6
	mt::Box{}, // 7
	mt::Box{}, // 8
	mt::Box{}, // 9
	mt::Box{} // shortcut
};

} // !Field

} // !p28

#endif