
#ifndef P28_PATHS_HPP_
#define P28_PATHS_HPP_

#include "Drivebase.hpp"

namespace p28 {

namespace Paths {

DrivebasePath gen_test();

DrivebasePath gen_yellowLane();
DrivebasePath gen_greenLane();
DrivebasePath add_greenLaneKnockCup(DrivebasePath currentPath);

DrivebasePath add_pingPong(DrivebasePath currentPath, DrivebaseState drvbState);

DrivebasePath gen_shortcut();

DrivebasePath hot_insert(DrivebasePath prevPath, DrivebasePath newPath);

}

} // !p28

#endif