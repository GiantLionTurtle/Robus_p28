
#ifndef P28_SOLVER_HPP_
#define P28_SOLVER_HPP_

#include "Drivebase.hpp"
#include "Field.hpp"



enum Legality { Cannot_go = 1, Can_go = 2, Unknown = 4 };
int is_move_legal(int sq_x, int sq_y, int move);
void set_legality(int sq_x, int sq_y, int move, int legality);
void init_legalityMatrix();

/*
    Trace system def here
*/

// Main function, solves the maze
struct Drivebase solve(struct Drivebase drvb);
struct Drivebase solve2(struct Drivebase drvb, bool& fail);
struct Drivebase failure(bool& fail);

//buzzer
void buzzerFin(void);




#endif