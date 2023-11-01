
#ifndef P28_FIELD_HPP_
#define P28_FIELD_HPP_

#include "Utils/Geometry.hpp"

namespace p28 {

namespace Field {

const mt::Vec2 zone1_topRight 	= mt::Vec2(0.122, 0.485);
const mt::Vec2 zone2_bottomLeft = mt::Vec2(0.122, 0.364);
const mt::Vec2 zone2_topRight 	= mt::Vec2(0.182, 0.485);
const mt::Vec2 zone3_bottomLeft = mt::Vec2(0.182, 0.364);
const mt::Vec2 zone5_bottomLeft = mt::Vec2(0.182, 0.123);
const mt::Vec2 zone6_topRight 	= mt::Vec2(0.302, 0.123);
const mt::Vec2 zone8_topRight 	= mt::Vec2(0.122, 0.123);
const mt::Vec2 zone9_bottomLeft = mt::Vec2(0.0, 0.123);

const mt::Line zone_1_to_2_line { .origin=zone1_topRight, 	.dir=zone1_topRight-zone2_bottomLeft };
const mt::Line zone_2_to_3_line { .origin=zone2_topRight, 	.dir=zone2_topRight-zone3_bottomLeft };
const mt::Line zone_5_to_6_line { .origin=zone5_bottomLeft, .dir=zone5_bottomLeft-zone6_topRight };
const mt::Line zone_8_to_9_line { .origin=zone8_topRight, 	.dir=zone8_topRight-zone9_bottomLeft };
 
constexpr int n_zones = 11;
const mt::Box zones_boxes[n_zones] {
	mt::Box{ .bottomLeft=mt::Vec2(0.0, 0.214), 		.topRight=mt::Vec2(0.122, 0.364) }, // 0
	mt::Box{ .bottomLeft=mt::Vec2(0.0, 0.364), 		.topRight=zone1_topRight}, // 1
	mt::Box{ .bottomLeft=zone2_bottomLeft, 			.topRight=zone2_topRight}, // 2
	mt::Box{ .bottomLeft=zone3_bottomLeft,		 	.topRight=mt::Vec2(0.302, 0.485)}, // 3
	mt::Box{ .bottomLeft=mt::Vec2(0.182, 0.244), 	.topRight=mt::Vec2(0.302, 0.364)}, // 4
	mt::Box{ .bottomLeft=zone5_bottomLeft,		 	.topRight=mt::Vec2(0.302, 0.244)}, // 5
	mt::Box{ .bottomLeft=mt::Vec2(0.182, 0.0), 		.topRight=zone6_topRight}, // 6
	mt::Box{ .bottomLeft=mt::Vec2(0.122, 0.0), 		.topRight=mt::Vec2(0.182, 0.123)}, // 7
	mt::Box{ .bottomLeft=mt::Vec2(0.0, 0.0), 		.topRight=zone8_topRight}, // 8
	mt::Box{ .bottomLeft=zone9_bottomLeft, 			.topRight=mt::Vec2(0.122, 0.206)}, // 9
	mt::Box{ .bottomLeft=mt::Vec2(0.122, 0.123), 	.topRight=mt::Vec2(0.182, 0.244)} // shortcut
};
const int kshortcutZone = 10;
} // !Field

} // !p28

#endif