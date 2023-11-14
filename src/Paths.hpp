
#ifndef P28_PATHS_HPP_
#define P28_PATHS_HPP_

#include <Utils/Vec2.hpp>
#include <Constants.hpp>

namespace p28 {

namespace Paths {

struct Arc {
	mt::Vec2 tengeantStart;
	mt::Vec2 end;
	float radius;
	float length;

	void print() const;
};

// Action that the drivebase can do (high level)
struct CheckPoint {
	mt::Vec2 targPos { 0.0 }; // Target position 
	mt::Vec2 targHeading { 0.0 };
	float targVel { 0.0 }; // Speed at the target position
	float maxVel { kMaxVel };
	unsigned int delay_before { 0 };
	bool turn_only { false };
	bool backward { false };
    int id { -1 }; // Usefull to do tasks at a specific point of a path

	CheckPoint() = default;
	CheckPoint(mt::Vec2 targPos_, mt::Vec2 targHeading_, 
					float targVel_ = 0.0, bool backward_ = false, float maxVel_ = kMaxVel, unsigned int delay_before_ = 0, int id_ = -1);

	static CheckPoint make_turn(mt::Vec2 targHeading_, unsigned int delay_before = 0, int id_ = -1);
};

struct Path {
	CheckPoint checkPoints[kMaxCheckPointForPath];
	
	unsigned int index { 0 };
	unsigned int size { 0 };

	CheckPoint& current() { return checkPoints[index]; }
	void add_checkPoint(CheckPoint checkpoint);
	void add_line(float distance);
	void add_turn(float turnAngle_rad);
	bool finished() const { return index >= size; }
};

Path fix(Path path);
Path hot_insert(Path prevPath, Path insert);

Path gen_test();

Arc arc_from_targetHeading(mt::Vec2 start, mt::Vec2 end, mt::Vec2 end_heading);

} // !Paths

} // !p28

#endif