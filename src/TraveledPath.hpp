
#ifndef P28_TRAVELEDPATH_HPP_
#define P28_TRAVELEDPATH_HPP_

#include <Arduino.h>
#include "LibRobus.h"
#include "Constants.hpp"

namespace p28 {


    void init_path();

    void add_move(int direction);

    int get_last_move();

    void delete_last_move();

    int opposite_move(int move);

    int retrace_last_move();

}

#endif