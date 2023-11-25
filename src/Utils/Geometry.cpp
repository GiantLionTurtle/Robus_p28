
#include "Geometry.hpp"

namespace p28 {

namespace mt {

// Are three points aranged in a ccw fashion?
bool threePoints_ccw(mt::Vec2 A, mt::Vec2 B, mt::Vec2 C)
{
	return (C.y-A.y) * (B.x-A.x) > (B.y-A.y) * (C.x-A.x);
}


Vec2 Line::line_intersection(Line const& l2) const
{
	mt::Vec2 originDiff = origin - l2.origin;
	float dirCross = mt::cross(dir, l2.dir);

	return origin + dir * mt::cross(l2.dir, originDiff) / dirCross;
}
Vec2 Line::closest_point(Vec2 const& pt) const
{
	return line_intersection(Line{.origin=pt, .dir=mt::cw_perpendicular(dir)});
}
Line Line::offset(Vec2 const& by) const
{
	return Line { .origin=origin+by, .dir=dir};
}
float Line::dist(mt::Vec2 const& pos) const
{
	// https://www.omnicalculator.com/math/triangle-height
	double a, b, c; // the side lengths of the triangle;
	mt::Vec2 ptA = origin;
	mt::Vec2 ptB = origin+dir;

	a = mt::distance(pos, ptA);
	b = mt::distance(ptA, ptB);
	c = mt::distance(pos, ptB);
	double h = 0.5 * sqrt((a + b + c) * (-a + b + c) * (a - b + c) * (a + b - c)) / b;
	
	return h;
}
float Line::dist_signed(mt::Vec2 const& pos) const
{
	mt::Vec2 ptA = origin;
	mt::Vec2 ptB = origin+dir;
	return threePoints_ccw(ptA, pos, ptB) ? dist(pos) : -dist(pos);
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