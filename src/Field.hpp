
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
	mt::Box{ .bottomLeft=mt::Vec2(0.0, 0.214), 		.topRight=mt::Vec2(0.122, 0.364) }, // 0
	mt::Box{ .bottomLeft=mt::Vec2(0.0, 0.364), 		.topRight=mt::Vec2(0.122, 0.485)}, // 1
	mt::Box{ .bottomLeft=mt::Vec2(0.122, 0.364), 	.topRight=mt::Vec2(0.182, 0.485)}, // 2
	mt::Box{ .bottomLeft=mt::Vec2(0.182, 0.364), 	.topRight=mt::Vec2(0.302, 0.485)}, // 3
	mt::Box{ .bottomLeft=mt::Vec2(0.182, 0.244), 	.topRight=mt::Vec2(0.302, 0.364)}, // 4
	mt::Box{ .bottomLeft=mt::Vec2(0.182, 0.123), 	.topRight=mt::Vec2(0.302, 0.244)}, // 5
	mt::Box{ .bottomLeft=mt::Vec2(0.182, 0.0), 		.topRight=mt::Vec2(0.302, 0.123)}, // 6
	mt::Box{ .bottomLeft=mt::Vec2(0.122, 0.0), 		.topRight=mt::Vec2(0.182, 0.123)}, // 7
	mt::Box{ .bottomLeft=mt::Vec2(0.0, 0.0), 		.topRight=mt::Vec2(0.122, 0.123)}, // 8
	mt::Box{ .bottomLeft=mt::Vec2(0.0, 0.123), 		.topRight=mt::Vec2(0.122, 0.206)}, // 9
	mt::Box{ .bottomLeft=mt::Vec2(0.122, 0.123), 	.topRight=mt::Vec2(0.182, 0.244)} // shortcut
};
const int kshortcutZone = 10;
} // !Field

} // !p28

#endif