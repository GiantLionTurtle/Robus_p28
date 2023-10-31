
#ifndef P28_PATHS_HPP_
#define P28_PATHS_HPP_

#include "Drivebase.hpp"

namespace p28 {

namespace Paths {

DrivebasePath gen_test();

DrivebasePath gen_yellowLine();
DrivebasePath gen_trapBal(DrivebaseState drvbState);
DrivebasePath gen_one_turn_path();

DrivebasePath path_hot_insert(DrivebasePath prevPath, DrivebasePath newPath);

}

} // !p28

#endif