
#ifndef P28_TRAVELEDPATH_HPP_
#define P28_TRAVELEDPATH_HPP_

#include <Arduino.h>
#include "LibRobus.h"
#include "Constants.hpp"

namespace p28 {


    void init_path();

    void add_move(struct Path traveled_path, int direction);

    int last_move(struct Path traveled_path);

    int retrace_last_move(struct Path traveled_path);

}

#endif