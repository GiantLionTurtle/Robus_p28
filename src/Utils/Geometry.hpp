
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
	float dist(mt::Vec2 const& pos);
	float dist_signed(mt::Vec2 const& pos);
};

template<typename T>
struct Box_any {
	Vec2_any<T> bottomLeft;
	Vec2_any<T> topRight;

	bool point_inside(mt::Vec2_any<T> pos) const
	{
		return pos.x > bottomLeft.x && pos.x < topRight.x && pos.y > bottomLeft.y && pos.y < topRight.y;
	}
	bool box_inside(Box_any<T> box) const
	{
		return point_inside(box.bottomLeft) && point_inside(box.topRight);
	}
};
using Box = Box_any<float>;
using i32Box = Box_any<int32_t>;

bool threePoints_ccw(mt::Vec2 A, mt::Vec2 B, mt::Vec2 C);

float to_radians(float degrees);
float to_degrees(float radians);

} // !mt

} // !p28

#endif