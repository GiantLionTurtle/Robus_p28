
#include "Geometry.hpp"

namespace p28 {

namespace mt {

Vec2 Line::line_intersection(Line const& l2) const
{
	mt::Vec2 originDiff = origin - l2.origin;
	float dirCross = mt::cross(dir, l2.dir);

	return origin + dir * mt::cross(l2.dir, originDiff) / dirCross;
}
bool Box::point_inside(mt::Vec2 pos) const
{
	return pos.x > bottomLeft.x && pos.x < topRight.x && pos.y > bottomLeft.y && pos.y < bottomLeft.y;    
}

// Are three points aranged in a ccw fashion?
bool threePoints_ccw(mt::Vec2 A, mt::Vec2 B, mt::Vec2 C)
{
	return (C.y-A.y) * (B.x-A.x) > (B.y-A.y) * (C.x-A.x);
}

float to_radians(float degrees)
{
	return degrees / 180 * PI;
}
float to_degrees(float radians)
{
	return radians * 180 / PI;
}

} // !mt

} // !p28