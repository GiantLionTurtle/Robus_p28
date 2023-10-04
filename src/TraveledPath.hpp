
#ifndef P28_TRAVELEDPATH_HPP_
#define P28_TRAVELEDPATH_HPP_

#include <Arduino.h>
#include "LibRobus.h"
#include "Constants.hpp"


void init_path();

void add_move(int direction);

int get_last_move();

int opposite_move(int move);

int retrace_last_move();

int stored_move(int ind);
int n_stored_moves();


#endif