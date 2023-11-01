
#ifndef P28_GEOMETRY_HPP_
#define P28_GEOMETRY_HPP_

#include "Vec2.hpp"

namespace p28 {

namespace mt {

struct Line {
	Vec2 origin;
	Vec2 dir;

	Vec2 line_intersection(Line const& l2) const;
    Vec2 closest_point(Vec2 const& pt) const;
    Line offset(Vec2 const& by) const;
};

struct Box {
    Vec2 bottomLeft;
    Vec2 topRight;

    bool point_inside(mt::Vec2 pos) const;
};

bool threePoints_ccw(mt::Vec2 A, mt::Vec2 B, mt::Vec2 C);

float to_radians(float degrees);
float to_degrees(float radians);

} // !mt

} // !p28

#endif