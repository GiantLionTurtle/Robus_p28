
#include "Paths.hpp"

namespace p28 {

namespace Paths {
    
DrivebasePath gen_test()
{
	DrivebasePath path;
	// path.add_checkPoint(PathCheckPoint(mt::Vec2(0.0, 0.5), mt::Vec2(0.0, 1.0), 0.0, 0, false));
	// path.add_checkPoint(PathCheckPoint(mt::Vec2(0.0, 20), mt::Vec2(0.0, 1.0)));

	// path.add_checkPoint(PathCheckPoint(mt::Vec2(0.0, 0.5), mt::Vec2(0.0, 1.0), 0.2));
	// path.add_checkPoint(PathCheckPoint(mt::Vec2(0.5, 0.5), mt::Vec2(0.0, -1.0), 0.2));
	// path.add_checkPoint(PathCheckPoint(mt::Vec2(0.5, 0.0), mt::Vec2(0.0, -1.0), 0.2));
	// path.add_checkPoint(PathCheckPoint(mt::Vec2(0.0, 0.0), mt::Vec2(0.0, 1.0), 0.0));

	path.add_checkPoint(PathCheckPoint(mt::Vec2(0.0, 1), mt::Vec2(0.0, 1.0), 0.08));
	path.add_checkPoint(PathCheckPoint(mt::Vec2(1, 1), mt::Vec2(0.0, -1.0), 0.08));
	path.add_checkPoint(PathCheckPoint(mt::Vec2(1, 0.0), mt::Vec2(0.0, -1.0), 0.08));
	path.add_checkPoint(PathCheckPoint(mt::Vec2(0.0, 0.0), mt::Vec2(0.0, 1.0), 0.08));

	return path;
}

DrivebasePath gen_yellowline()
{
	DrivebasePath path;
    path.add_checkPoint(PathCheckPoint(mt::Vec2(1.0, 1.0), mt::Vec2(1.0, 0.0), 0.6));
    path.add_checkPoint(PathCheckPoint(mt::Vec2(1.0, -1.0), mt::Vec2(0.0, -1.0), 0.6));
	path.add_checkPoint(PathCheckPoint(mt::Vec2(-1.0, -1.0), mt::Vec2(-1.0, 0.0), 0.6));
	path.add_checkPoint(PathCheckPoint(mt::Vec2(-1.0, 1.0), mt::Vec2(0.0, 1.0), 0.0));
	
	return path;
}

DrivebasePath gen_trapBal(DrivebaseState drvbState)
{
	DrivebasePath path;
	path.add_checkPoint(PathCheckPoint(mt::Vec2(0.0,-1.0), mt::Vec2(0.0, -1)));
	path.add_checkPoint(PathCheckPoint(mt::Vec2(-1.0, 1.0), mt::Vec2(0.0, 1.0), 0.0));

	return path;
}

DrivebasePath gen_one_turn_path()
{
	DrivebasePath path;
	path.add_checkPoint(PathCheckPoint(mt::Vec2(0.0,1.0), mt::Vec2(0.0, 1.0)));
	path.add_checkPoint(PathCheckPoint(mt::Vec2(0.5, 1.0), mt::Vec2(0.0, -1.0)));
	path.add_checkPoint(PathCheckPoint(mt::Vec2(0.5, 0.5), mt::Vec2(0.0, -1.0)));

	return path;
}

DrivebasePath path_hot_insert(DrivebasePath prevPath, DrivebasePath newPath)
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