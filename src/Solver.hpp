
#ifndef P28_SOLVER_HPP_
#define P28_SOLVER_HPP_

#include "Drivebase.hpp"

namespace p28 {

/*
    legality matrix def here
*/
enum Legality { Cannot_go = 1, Can_go = 2, Unknown = 4 };

// returns the Legality of a move (LEFT, RIGHT, FRONT or REAR)
Legality is_legal_move(Drivebase drvb, int move);

// Sets the legality matrix after discovering that a move is legal
void set_legality(bool legal, int x, int y, int move);

/*
    Trace system def here
*/

// Main function, solves the maze
Drivebase solve(Drivebase drvb);

} // !p28



#endif