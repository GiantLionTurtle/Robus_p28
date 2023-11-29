
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

	CheckPoint() = default;
	CheckPoint(mt::Vec2 targPos_, mt::Vec2 targHeading_, 
					float targVel_ = 0.0, bool backward_ = false, float maxVel_ = kMaxVel, unsigned int delay_before_ = 0);

	static CheckPoint make_turn(mt::Vec2 targHeading_, unsigned int delay_before = 0, mt::Vec2 targPos_ = {0.0});
};

struct Path {
	CheckPoint checkPoints[kMaxCheckPointForPath];
	
	unsigned int index { 0 };
	unsigned int size { 0 };

	CheckPoint& current() { return checkPoints[index]; }
	void add_checkPoint(CheckPoint checkpoint);
	void add_line(float distance, float targVel_ = 0.0, float maxVel_ = kMaxVel, unsigned int delay_before_ = 0);
	void add_turn(float turnAngle_rad);
	bool finished() const { return index >= size; }
	void reset() { index = 0; size = 0; }
};

void fix(Path const& src, Path& dst);
void hot_insert(Path const& prevPath, Path& insert);
void deep_copy(Path const& src, Path& dst);

void gen_getToLine(mt::Vec2 currPos, mt::Vec2 currHeading, int target_color, Path& dst);
void gen_drop(mt::Vec2 currPos, mt::Vec2 currHeading, int target_color, Path& dst);
void gen_reset(mt::Vec2 currPos, mt::Vec2 currHeading, Path& dst);

void gen_test();
void gen_realSearchPath(mt::Vec2 currPos, mt::Vec2 currHeading, Path& dst);
void gen_searchPath(mt::Vec2 currPos, mt::Vec2 currHeading, Path& dst);

Arc arc_from_targetHeading(mt::Vec2 start, mt::Vec2 end, mt::Vec2 end_heading);

} // !Paths

} // !p28

#endif