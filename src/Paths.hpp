
#ifndef P28_PATHS_HPP_
#define P28_PATHS_HPP_

#include "Subsystems/Drivebase.hpp"

namespace p28 {

namespace Paths {

DrivebasePath gen_test();

DrivebasePath hot_insert(DrivebasePath prevPath, DrivebasePath newPath);

}

} // !p28

#endif