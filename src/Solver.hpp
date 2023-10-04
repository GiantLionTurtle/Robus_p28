
#ifndef P28_SOLVER_HPP_
#define P28_SOLVER_HPP_

#include "Drivebase.hpp"
#include "Field.hpp"

namespace p28 {

// Get the index in the LegalityMatrix for a given
// square
int legalityIndex(int x, int y);

enum Legality { Cannot_go = 1, Can_go = 2, Unknown = 4 };

// returns the Legality of a move (LEFT, RIGHT, FRONT or REAR)
Legality is_legal_move(int sq_x, int sq_y, int move);

// Sets the legality matrix after discovering that a move is legal
void set_legality(bool legal, int sq_x, int sq_y, int move);

void init_legalityMatrix();

/*
    Trace system def here
*/

// Main function, solves the maze
Drivebase solve(Drivebase drvb);
Drivebase solve2(Drivebase drvb);

} // !p28



#endif