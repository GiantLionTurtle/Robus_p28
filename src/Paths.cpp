
#include "Paths.hpp"
#include "Field.hpp"

namespace p28 {

namespace Paths {
    
DrivebasePath gen_test()
{
	Serial.println("Gen test");
	DrivebasePath path;
	// path.add_checkPoint(PathCheckPoint(mt::Vec2(0.0, 0.5), mt::Vec2(0.0, 1.0), 0.0, 0, false));
	path.add_checkPoint(PathCheckPoint(mt::Vec2(0.0, 20), mt::Vec2(0.0, 1.0)));

	// path.add_checkPoint(PathCheckPoint(mt::Vec2(0.0, 0.5), mt::Vec2(0.0, 1.0), kEndSegmentVel));
	// path.add_checkPoint(PathCheckPoint(mt::Vec2(0.5, 0.5), mt::Vec2(0.0, -1.0), kEndSegmentVel));
	// path.add_checkPoint(PathCheckPoint(mt::Vec2(0.5, 0.0), mt::Vec2(0.0, -1.0), kEndSegmentVel));
	// path.add_checkPoint(PathCheckPoint(mt::Vec2(0.0, 0.0), mt::Vec2(0.0, 1.0), 0.0));

	// path.add_checkPoint(PathCheckPoint(mt::Vec2(0.0, 1), mt::Vec2(0.0, 1.0), kEndSegmentVel));
	// path.add_checkPoint(PathCheckPoint(mt::Vec2(1, 1), mt::Vec2(0.0, -1.0), kEndSegmentVel));
	// path.add_checkPoint(PathCheckPoint(mt::Vec2(1, 0.0), mt::Vec2(0.0, -1.0), kEndSegmentVel));
	// path.add_checkPoint(PathCheckPoint(mt::Vec2(0.0, 0.0), mt::Vec2(0.0, 1.0), 0.0));

	path.add_checkPoint(PathCheckPoint::make_turn(mt::Vec2(0.0, -1.0)));
	path.add_checkPoint(PathCheckPoint(mt::Vec2(0.0, 1), mt::Vec2(0.0, 1.0), kEndSegmentVel, true));
	path.add_checkPoint(PathCheckPoint(mt::Vec2(1, 1), mt::Vec2(0.0, -1.0), kEndSegmentVel, true));
	path.add_checkPoint(PathCheckPoint(mt::Vec2(1, 0.0), mt::Vec2(0.0, -1.0), kEndSegmentVel, true));
	path.add_checkPoint(PathCheckPoint(mt::Vec2(0.0, 0.0), mt::Vec2(0.0, 1.0), 0.0, true));
	path.add_checkPoint(PathCheckPoint::make_turn(mt::Vec2(0.0, 1.0)));


	return path;
}

DrivebasePath gen_yellowLane()
{
	DrivebasePath path;
	Serial.println("Beginning yellow Path");

	
	path.add_checkPoint(PathCheckPoint(mt::Vec2(0.4572 , 3.64), mt::Vec2(0.0, 1.0)));
	path.add_checkPoint(PathCheckPoint(mt::Vec2(1.319 , 4.4696), mt::Vec2(1.0, 0.0), 0.15));
	path.add_checkPoint(PathCheckPoint(mt::Vec2(1.799 , 4.4696), mt::Vec2(1.0,0.0), 0.0f, false, 0.2));
	path.add_checkPoint(PathCheckPoint(mt::Vec2(2.651 , 3.64), mt::Vec2(0.0,-1.0)));
	path.add_checkPoint(PathCheckPoint(mt::Vec2(2.651 , 1.2192), mt::Vec2(0.0,-1.0)));
	path.add_checkPoint(PathCheckPoint(mt::Vec2(2.75 , 1.05), mt::Vec2(-1.0 , 0.0)));
	path.add_checkPoint(PathCheckPoint(mt::Vec2(-1.0 , 1.05), mt::Vec2(-1.0 , 0.0)));
	//Hardcoded line following section
	// path.add_checkPoint(PathCheckPoint::make_turn(Field::yellow_follow_line1.dir));
	// path.add_checkPoint(PathCheckPoint(Field::yellow_follow_line2.origin, Field::yellow_follow_line1.dir));
	// path.add_checkPoint(PathCheckPoint::make_turn(Field::yellow_follow_line2.dir));
	// path.add_checkPoint(PathCheckPoint(Field::yellow_follow_line3.origin, Field::yellow_follow_line2.dir));
	// path.add_checkPoint(PathCheckPoint::make_turn(Field::yellow_follow_line3.dir));
	// path.add_checkPoint(PathCheckPoint(mt::Vec2(0.305 , 1.22), Field::yellow_follow_line3.dir));
	// path.add_checkPoint(PathCheckPoint::make_turn(mt::Vec2(0.0 , 1.0)));
	 path.add_checkPoint(PathCheckPoint(mt::Vec2(00.762 , 3.64), mt::Vec2(0.0, 1.0))); //startting position of the short cute line
	return path;

}
DrivebasePath gen_greenLane()
{
	Serial.println("Beginning green path");
	DrivebasePath path;
	bool backward = true;
	float zone_678_maxSpeed = 0.8;

#ifdef RACE_MODE // Do not turn backward if we are in race mode!
	// Start backward to knock the cup!
	backward = false;

	zone_678_maxSpeed = kMaxVel; // Slower when we might have a ball to deal with
#endif
	
	path.add_checkPoint(PathCheckPoint(mt::Vec2(0.762 , 3.64), mt::Vec2(0.0 , 1.0)));
	path.add_checkPoint(PathCheckPoint(mt::Vec2(1.319 , 4.115), mt::Vec2(1.0 , 0.0), 0.15));
	path.add_checkPoint(PathCheckPoint(mt::Vec2(1.799 , 4.115), mt::Vec2(1.0 , 0.0),  0.0f, false, 0.2));
	path.add_checkPoint(PathCheckPoint(mt::Vec2(2.186 , 3.64), mt::Vec2(0.0 , -1.0)));
	path.add_checkPoint(PathCheckPoint::make_turn(mt::Vec2(0.1 ,1.0)));
	path.add_checkPoint(PathCheckPoint(mt::Vec2(2.236, 1.5), mt::Vec2(0.0,-1.0), kEndSegmentVel, backward));
	path.add_checkPoint(PathCheckPoint(mt::Vec2(2.236,-2.0), mt::Vec2(0.0,-1.0), kEndSegmentVel, backward, 0.2));
	// path.add_checkPoint(PathCheckPoint::make_turn(Field::green_follow_line1.dir));
	// path.add_checkPoint(PathCheckPoint(Field::green_follow_line2.origin, Field::green_follow_line1.dir));
	// path.add_checkPoint(PathCheckPoint::make_turn(Field::green_follow_line2.dir));
	// path.add_checkPoint(PathCheckPoint(Field::green_follow_line3.origin, Field::green_follow_line2.dir));
	// path.add_checkPoint(PathCheckPoint::make_turn(Field::green_follow_line3.dir));
	// path.add_checkPoint(PathCheckPoint(mt::Vec2(0.305 , 1.22), Field::green_follow_line3.dir));
	// path.add_checkPoint(PathCheckPoint::make_turn(mt::Vec2(0.0 , 1.0)));
	path.add_checkPoint(PathCheckPoint(mt::Vec2(0.762 , 3.64), mt::Vec2(0.0, 1.0))); //position of the shortcut line beginning

	return path;
}
DrivebasePath add_greenLaneKnockCup(DrivebasePath currentPath)
{
	DrivebasePath knockCupPath;
	knockCupPath.add_checkPoint(PathCheckPoint::make_turn(mt::Vec2(0.0, -1.0)));
	knockCupPath.add_checkPoint(PathCheckPoint::make_turn(mt::Vec2(0.1, -1.0)));


	currentPath = hot_insert(currentPath, knockCupPath);

	// We turned around so we must go forward
	for(int i = knockCupPath.size; i < currentPath.size; ++i) {
		currentPath.segments[i].backward = false;
	}
	return currentPath;
}

DrivebasePath add_pingPong(DrivebasePath currentPath, DrivebaseState drvbState)
{
	DrivebasePath swervePath;
	
	// The robot should be oriented ~ (-1, 0) at this point

	// Orient the cup dropper over the ball
	swervePath.add_checkPoint(PathCheckPoint::make_turn(mt::rotate(drvbState.heading, mt::to_radians(15))));

	// Wait for the cup to drop then go perpendicular to the line
	swervePath.add_checkPoint(PathCheckPoint::make_turn(mt::rotate(drvbState.heading, mt::to_radians(90)), 1000));

	// Add two arcs to get back to the line
	swervePath.add_checkPoint(PathCheckPoint(drvbState.pos + drvbState.heading*.15 + mt::cw_perpendicular(drvbState.heading), -drvbState.heading ));

	// combine the paths 
	currentPath = hot_insert(currentPath, swervePath);

	// This part of the original path was slower to allow the robot to break
	// in time for the wall, speed it up after the ball is dealt with
	if(currentPath.size > swervePath.size) {
		currentPath.segments[swervePath.size].maxVel = kMaxVel;
	}
}

DrivebasePath gen_shortcut()
{

}

DrivebasePath hot_insert(DrivebasePath prevPath, DrivebasePath newPath)
{
	int prevSize = prevPath.size;
	int newSize = newPath.size;
	int limit = prevSize+newSize;
	for(int i = newSize; i<limit;i++)
	{
		newPath.segments[i] = prevPath.segments[prevPath.index+(i-newSize)];
		newPath.size++;
	}
	return newPath;

}

}

} // !p28