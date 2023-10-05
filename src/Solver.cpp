
#include "Solver.hpp"
#include "LibRobus.h"
#include "TraveledPath.hpp"


#define LEFT_WALL_BIT_MASK (1 | 2 | 4)
#define REAR_WALL_BIT_MASK (8 | 16 | 32)
#define REAR_WALL_BIT_OFFSET 3

// Uncomment to go back at the begining of
// the maze when finished solving
// #define GO_BACK_TO_BEGINING




// Each char represents the left and bottom "walls" of 
// a case
static char LegalityMatrix[(kFieldWidth+1)*(kFieldHeight+1)];

int legalityIndex(int sq_x, int sq_y)
{
	return sq_y * (kFieldWidth+1) + sq_x;
}

int is_move_legal(int sq_x, int sq_y, int move)
{
	switch(move) {
	case LEFT:
		return LegalityMatrix[legalityIndex(sq_x, sq_y)] & ~REAR_WALL_BIT_MASK;
	case RIGHT:
		return is_move_legal(sq_x+1, sq_y, LEFT);
	case FRONT:
		return is_move_legal(sq_x, sq_y+1, REAR);
	case REAR:
		return (LegalityMatrix[legalityIndex(sq_x, sq_y)] & ~LEFT_WALL_BIT_MASK) >> REAR_WALL_BIT_OFFSET;
	default:
		return Legality::Cannot_go;
	}
}
void set_legality(int sq_x, int sq_y, int move, int legality)
{
	int ind;
	switch(move) {
	case LEFT:
			ind = legalityIndex(sq_x, sq_y);
			LegalityMatrix[ind] &= ~LEFT_WALL_BIT_MASK;
			LegalityMatrix[ind] |= legality;
		break;
	case RIGHT:
			set_legality(sq_x+1, sq_y, LEFT, legality);
		break;
	case FRONT:
			set_legality(sq_x, sq_y+1, REAR, legality);
		break;
	case REAR:
			ind = legalityIndex(sq_x, sq_y);
			LegalityMatrix[ind] &= ~REAR_WALL_BIT_MASK;
			LegalityMatrix[legalityIndex(sq_x, sq_y)] |= (legality << REAR_WALL_BIT_OFFSET);
		break;
	default:
		break;
	}
}


void init_legalityMatrix()
{
	for(int i = 0; i < kFieldWidth+1; ++i) {
		for(int j = 0; j < kFieldHeight+1; ++j) {
			set_legality(i, j, LEFT, Legality::Unknown);
			set_legality(i, j, REAR, Legality::Unknown);
		}
	}
	// Black walls in the middle
	for(int i = 1; i < 10; i+=2) {
		set_legality(1, i, LEFT, Legality::Cannot_go);
		set_legality(1, i, RIGHT, Legality::Cannot_go);
	}
	// Outer walls
	for(int i = 0; i < kFieldWidth+1; ++i) {
		set_legality(i, 0, REAR, Legality::Cannot_go);
		set_legality(i, kFieldHeight-1, FRONT, Legality::Cannot_go);
	}
	for(int i = 0; i < kFieldHeight+1; ++i) {
		set_legality(0, i, LEFT, Legality::Cannot_go);
		set_legality(kFieldWidth-1, i, RIGHT, Legality::Cannot_go);
	}
}

struct Drivebase try_move(struct Drivebase drvb, int move, int illegal_move, int try_n_squares, bool& success)
{
	if(move == illegal_move) {
		success = false;
		return drvb;
	}
	int legal = is_move_legal(drvb.sq_x, drvb.sq_y, move);

	if(legal == Legality::Can_go) {
		drvb = move_to_square(drvb, move, 1);
		success = true;
		delay(kDecelerationDelay);
	} else if(legal == Legality::Unknown) {
		bool wall = false;
		int init_sq_x = drvb.sq_x;
		int init_sq_y = drvb.sq_y;

		drvb = move_to_square_or_detect(drvb, move, try_n_squares, wall);
		
		set_legality(init_sq_x, init_sq_y, move, wall ? Legality::Cannot_go : Legality::Can_go);
		success = !wall;
		delay(kDecelerationDelay);
	} else {
		success = false;
	}
	return drvb;
}
struct Drivebase step(struct Drivebase drvb, bool& fail)
{
	int auto_fail = opposite_move(get_last_move());

	bool success = false;
	// Shoot for the stars, try to move forward to the en of the maze
	int init_sq_y = drvb.sq_y;
	drvb = try_move(drvb, FRONT, auto_fail, 1, success);
	if(success) {
		add_move(FRONT);
		set_legality(drvb.sq_x, drvb.sq_y, opposite_move(FRONT), Legality::Cannot_go);
		return drvb;
	}
	drvb = try_move(drvb, LEFT, auto_fail, 1, success);
	if(success) {
		add_move(LEFT);
		set_legality(drvb.sq_x, drvb.sq_y, opposite_move(LEFT), Legality::Cannot_go);
		return drvb;
	}
	drvb = try_move(drvb, RIGHT, auto_fail, 1, success);
	if(success) {
		add_move(RIGHT);
		set_legality(drvb.sq_x, drvb.sq_y, opposite_move(RIGHT), Legality::Cannot_go);
		return drvb;
	}
	drvb = try_move(drvb, REAR, auto_fail, 1, success);
	if(success) {
		add_move(REAR);
		set_legality(drvb.sq_x, drvb.sq_y, opposite_move(REAR), Legality::Cannot_go);
		return drvb;
	}

	// Manage back
	int trace_back = retrace_last_move();
	if(trace_back == -1){
		fail = true;
		return drvb; // give up;
	}

	drvb = move_to_square(drvb, trace_back, 1);
	// Update Legality matrix
	set_legality(drvb.sq_x, drvb.sq_y, opposite_move(trace_back), Legality::Cannot_go);

	return drvb;
}
struct Drivebase solve2(struct Drivebase drvb, bool& fail)
{
	int n_stored = n_stored_moves();
	Serial.print("N stored: ");
	Serial.println(n_stored);
	// if(n_stored <= 0) 
	// { // Maze is not solved yet
		while(drvb.sq_y != 9) 
		{
			drvb = step(drvb, fail);
		}
	// // } 
	// else 
	// {
	// 	for(int i = 0; i < n_stored; ++i) 
		// {
	// 		drvb = move_to_square(drvb, stored_move(i), 1);
	// 	}
	// }

#ifndef GO_BACK_TO_BEGINING
	delay(1000);
	n_stored = n_stored_moves();
	for(int i = n_stored-1; i >= 0; --i) {
		// don't use retrace_last_move to remember the optimal path
		// for next time
		drvb = move_to_square(drvb, opposite_move(stored_move(i)), 1);
	}
#endif

	return drvb;
}
void buzzerFin(void)
{
	AX_BuzzerON(1000, 200);
	delay(400);
	AX_BuzzerON(800, 200);
	delay(400);
	AX_BuzzerON(800, 150);
	delay(200);
	AX_BuzzerON(1000, 150);
	delay(200);
	AX_BuzzerON(1500, 150);
	delay(200);
	AX_BuzzerON(240, 400);
	delay(600);
	AX_BuzzerON(120, 400);
	delay(600);
}
  struct Drivebase failure(bool& fail)
{
	if(fail = true){
		buzzerFin(); //buzzer function for fail
	}
}


