
#include "Paths.hpp"
#include "Field.hpp"

namespace p28 {

namespace Paths {
    
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