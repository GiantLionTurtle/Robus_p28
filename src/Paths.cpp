
#include "Paths.hpp"
#include "Field.hpp"

namespace p28 {

namespace Paths {

Arc arc_from_targetHeading(mt::Vec2 start, mt::Vec2 end, mt::Vec2 end_heading);

Path fix(Path path)
{
	if(path.finished())
		return Path();
	Path newPath;
	newPath.add_checkPoint(path.checkPoints[0]);

	for(unsigned int i = 1; i < path.size; ++i) {
		Arc arc = arc_from_targetHeading(path.checkPoints[i-1].targPos, path.checkPoints[i].targPos, path.checkPoints[i].targHeading);
		if(mt::signed_angle(arc.tengeantStart, path.checkPoints[i-1].targHeading) > mt::to_radians(10)) {
			newPath.checkPoints[i-1].targVel = 0.0; // Stop the robot before adjusting the angle
			newPath.add_checkPoint(CheckPoint::make_turn(arc.tengeantStart));
		}
		newPath.add_checkPoint(path.checkPoints[i]);
	}
	return newPath;
}
Path hot_insert(Path prevPath, Path insert)
{
	for(int i = prevPath.index; i < prevPath.size; ++i) {
		insert.add_checkPoint(prevPath.checkPoints[i]);
	}
	return insert;
}

Arc arc_from_targetHeading(mt::Vec2 start, mt::Vec2 end, mt::Vec2 end_heading)
{
	// See fig.3
	if(mt::epsilon_equal(end_heading, mt::normalize(end-start), 0.01f)) { // Protect against overflow and div by 0
		return Arc{ .tengeantStart=end_heading, .end=end, .radius=kInfinity, .length=mt::distance(end, start) };
	}

	Arc arcTGH;
	arcTGH.end = end;

	mt::Vec2 vecStartEnd = end - start;
	mt::Vec2 orientationCenterEnd = { end_heading.y,-end_heading.x };
	mt::Line line_end_center { .origin=end, .dir=orientationCenterEnd};
	mt::Line line_mid_startend_center {.origin=vecStartEnd/2.0f+start, .dir={-vecStartEnd.y, vecStartEnd.x}};

	mt::Vec2 center = line_end_center.line_intersection(line_mid_startend_center);

	mt::Vec2 center_to_end = end-center;
	mt::Vec2 center_to_start = start-center;

	float angleRadius_CenterEnd = angle(orientationCenterEnd,vecStartEnd);
	arcTGH.radius = (magnitude(vecStartEnd)/2)/(cos(angleRadius_CenterEnd));

	if(abs(arcTGH.radius) >= kInfinity) {
		return Arc{ .tengeantStart=end_heading, .end=end, .radius=kInfinity, .length=mt::distance(end, start) };
	}

	float circumference = 2*PI*abs(arcTGH.radius);
	float angleArc = mt::angle(center_to_start, center_to_end);

	arcTGH.tengeantStart = mt::normalize(mt::ccw_perpendicular(center_to_start));

	if(arcTGH.radius < 0.0f) { // If we are going clock wise
		angleArc = 2*PI-angleArc; // The angle is the complement of ccw angle
		arcTGH.tengeantStart = -arcTGH.tengeantStart; // Reverse start heading;		
	}

	arcTGH.length = circumference * (angleArc/(2*PI));

	return arcTGH;
}

void Arc::print() const
{
	mt::print(tengeantStart, 4);
	Serial.print(",  ");
	mt::print(end, 4);
	Serial.print(",  ");
	Serial.print(radius, 4);
	Serial.print(",  ");
	Serial.println(length, 4);
}

CheckPoint::CheckPoint(mt::Vec2 targPos_, mt::Vec2 targHeading_,
								float targVel_, bool backward_, 
								float maxVel_, unsigned int delay_before_, int id_)
	: targPos(targPos_)
	, targHeading(mt::normalize(targHeading_))
	, targVel(targVel_)
	, maxVel(maxVel_)
	, delay_before(delay_before_)
	, backward(backward_)
	, id(id_)
{
	
}
CheckPoint CheckPoint::make_turn(mt::Vec2 targHeading_, unsigned int delay_before, int id_)
{	
	CheckPoint out;
	out.targHeading = mt::normalize(targHeading_);
	out.turn_only = true;
	out.delay_before = 0;
	out.id = id_;
	return out;
}
void Path::add_checkPoint(CheckPoint checkPoint)
{
	if(size >= kMaxCheckPointForPath) {
		Serial.println("Path overflow!");
		return;
	}
	checkPoints[size] = checkPoint;
	size++;
}

void Path::add_line(float distance)
{
	if(size<=0){
		Serial.println("Error");
	}else 
	{
		CheckPoint lineCheckPoint (checkPoints[size-1].targPos + checkPoints[size-1].targHeading*distance, checkPoints[size-1].targHeading);
		add_checkPoint(lineCheckPoint);
	}
}

void Path::add_turn(float turnAngle_rad)
{
	if(size<=0){
		Serial.println("Error");
	}else 
	{
		CheckPoint turnCheckPoint = CheckPoint::make_turn(mt::rotate(checkPoints[size-1].targHeading, turnAngle_rad));
		add_checkPoint(turnCheckPoint);
	}
}

Path gen_test ()
{
	Path path;
	Serial.println("Gen test");
	path.add_checkPoint(CheckPoint(mt::Vec2(0.0, 0.0), mt::Vec2(0.0, 1.0)));
	path.add_line(.50);
	path.add_turn(mt::to_radians(135));
	path.add_line(.25);
	path.add_turn(mt::to_radians(135));
	path.add_line(0.30);
	return path;//fix(path);
}

} // !Paths

} // !p28