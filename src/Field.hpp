
#ifndef P28_FIELD_HPP_
#define P28_FIELD_HPP_

#include "Utils/Geometry.hpp"

namespace p28 {

namespace Field {

const float Foot = 0.3048;
//problème d'un facteur 10 avec les coordonnées des box
const mt::Vec2 zone1_topRight 	= mt::Vec2(0.122, 0.485);
const mt::Vec2 zone2_bottomLeft = mt::Vec2(0.122, 0.364);
const mt::Vec2 zone2_topRight 	= mt::Vec2(0.182, 0.485);
const mt::Vec2 zone3_bottomLeft = mt::Vec2(0.182, 0.364);
const mt::Vec2 zone5_bottomLeft = mt::Vec2(0.182, 0.123);
const mt::Vec2 zone6_topRight 	= mt::Vec2(3.117, 1.23);
const mt::Vec2 zone8_topRight 	= mt::Vec2(0.122, 0.123);
const mt::Vec2 zone9_bottomLeft = mt::Vec2(0.0, 0.123);

const mt::Line zone_1_to_2_line { .origin=zone1_topRight, 	.dir=zone1_topRight-zone2_bottomLeft };
const mt::Line zone_2_to_3_line { .origin=zone2_topRight, 	.dir=zone2_topRight-zone3_bottomLeft };
const mt::Line zone_5_to_6_line { .origin=zone5_bottomLeft, .dir=zone5_bottomLeft-zone6_topRight };
const mt::Line zone_8_to_9_line { .origin=zone8_topRight, 	.dir=zone8_topRight-zone9_bottomLeft };

const mt::Line yellow_follow_line1 { .origin=mt::Vec2(7*Foot , 4*Foot),  .dir=mt::Vec2(-1.0 , -1.0)};
const mt::Line yellow_follow_line2 { .origin=mt::Vec2(6*Foot, 1*Foot),  .dir=mt::Vec2(-1.0 , 0.0)};																																																
const mt::Line yellow_follow_line3 { .origin=mt::Vec2(4*Foot, 1*Foot),  .dir=mt::Vec2(-1.0 , 1.0)};

const mt::Vec2 yellow_startPos { 1.5*Foot , 3.4876 };

const mt::Line green_follow_line1 { .origin=mt::Vec2(8.0*Foot , 4*Foot),  .dir=mt::Vec2(-1.0 , -1.0)};
const mt::Line green_follow_line2 { .origin=mt::Vec2(6*Foot, 3*Foot),  .dir=mt::Vec2(-1.0 , 0.0)};																																																
const mt::Line green_follow_line3 { .origin=mt::Vec2(4*Foot, 3*Foot),  .dir=mt::Vec2(-1.0 , 1.0)};

const mt::Vec2 green_startPos { 2.5*Foot , 3.4876 };

constexpr int n_zones = 11;
const mt::Box zones_boxes[n_zones] {
	mt::Box{ .bottomLeft=mt::Vec2(0.0, 0.214), 		.topRight=mt::Vec2(0.122, 0.364) }, // 0
	mt::Box{ .bottomLeft=mt::Vec2(0.0, 0.364), 		.topRight=zone1_topRight}, // 1
	mt::Box{ .bottomLeft=zone2_bottomLeft, 			.topRight=zone2_topRight}, // 2
	mt::Box{ .bottomLeft=zone3_bottomLeft,		 	.topRight=mt::Vec2(0.302, 0.485)}, // 3
	mt::Box{ .bottomLeft=mt::Vec2(0.182, 0.244), 	.topRight=mt::Vec2(0.302, 0.364)}, // 4
	mt::Box{ .bottomLeft=zone5_bottomLeft,		 	.topRight=mt::Vec2(0.302, 0.244)}, // 5
	mt::Box{ .bottomLeft=mt::Vec2(2.201, -3.00), 		.topRight=zone6_topRight}, // 6
	mt::Box{ .bottomLeft=mt::Vec2(1.22, 0.0), 		.topRight=mt::Vec2(2.201, 1.23)}, // 7
	mt::Box{ .bottomLeft=mt::Vec2(0.0, 0.0), 		.topRight=zone8_topRight}, // 8
	mt::Box{ .bottomLeft=zone9_bottomLeft, 			.topRight=mt::Vec2(0.122, 0.206)}, // 9
	mt::Box{ .bottomLeft=mt::Vec2(0.122, 0.123), 	.topRight=mt::Vec2(0.182, 0.244)} // shortcut
};
const int kshortcutZone = 10;
} // !Field

} // !p28

#endif