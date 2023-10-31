
#ifndef P28_PATHS_HPP_
#define P28_PATHS_HPP_

#include "Drivebase.hpp"

namespace p28 {

namespace Paths {

DrivebasePath gen_test();

DrivebasePath gen_yellowLine();
DrivebasePath gen_trapBal(DrivebaseState drvbState);
DrivebasePath gen_one_cw_turn_path();

}

} // !p28

#endif