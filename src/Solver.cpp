
#include "Solver.hpp"
#include "LibRobus.h"
#include "TraveledPath.hpp"


#define LEFT_WALL_BIT_MASK (1 | 2 | 4)
#define REAR_WALL_BIT_MASK (8 | 16 | 32)
#define REAR_WALL_BIT_OFFSET 3

// Uncomment to go back at the begining of
// the maze when finished solving
#define GO_BACK_TO_BEGINING



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
String to_string(int move)
{
	switch(move) {
	case LEFT: return String("LEFT");
	case RIGHT: return String("RIGHT");
	case FRONT: return String("FRONT");
	case REAR: return String("REAR");
	}
	return String("wot");
}

int how_many_front(Drivebase drvb)
{
	int i;
	for(i = drvb.sq_y; i < kFieldHeight; ++i && i - drvb.sq_y < 4) {
		if(is_move_legal(drvb.sq_x, i, FRONT) == Legality::Cannot_go)
			break;
	}
	return i - drvb.sq_y;
}
void add_virtual_walls_helper(Drivebase end_drvb, int move, int n_squares)
{
	int end_x = end_drvb.sq_x;
	int end_y = end_drvb.sq_y;
	int opp_move = opposite_move(move);
	switch(move) {
	case FRONT:
		for(int i = 0; i < n_squares; ++i) {
			set_legality(end_x, end_y-i, opp_move, Legality::Cannot_go);
		}
		break;
	case REAR:
		for(int i = 0; i < n_squares; ++i) {
			set_legality(end_x, end_y+i, opp_move, Legality::Cannot_go);
		}
		break;
	case LEFT:
		for(int i = 0; i < n_squares; ++i) {
			set_legality(end_x+i, end_y, opp_move, Legality::Cannot_go);
		}
		break;
	case RIGHT:
		for(int i = 0; i < n_squares; ++i) {
			set_legality(end_x-i, end_y, opp_move, Legality::Cannot_go);
		}
		break;
	default:
		break;
	}
}

Drivebase do_move(Drivebase drvb, int move, int n_squares, bool& wall)
{
	// Serial.print("Do move ");
	// Serial.print(drvb.sq_x);
	// Serial.print(",  ");
	// Serial.print(drvb.sq_y);
	// Serial.print(" :: ");
	// Serial.print(to_string(move));
	// Serial.print(" [");
	// Serial.print(n_squares);
	// Serial.println("]");

	int done_squares = 0;
	drvb = move_to_square_or_detect(drvb, move, n_squares, done_squares);
	// Serial.print("Done squares: ");
	// Serial.println(done_squares);

	for(int i = 0; i < done_squares; ++i) {
		add_move(move);
	}
	wall = done_squares < n_squares;
	if(wall) {
		// Serial.println("WALL!");
		set_legality(drvb.sq_x, drvb.sq_y, move, Legality::Cannot_go); // Actual wall
	}
	add_virtual_walls_helper(drvb, move, done_squares); // virtual wall to avoid going
	return drvb;
}
Drivebase do_move_1_square(Drivebase drvb, int move, bool& wall)
{
	if(is_move_legal(drvb.sq_x, drvb.sq_y, move) != Legality::Cannot_go) {
		drvb = do_move(drvb, move, 1, wall);
	} else {
		wall = true;
	}
	return drvb;
}
Drivebase step(Drivebase drvb, bool& fail)
{
	int n_steps = how_many_front(drvb);
	bool wall = true;
	if(n_steps != 0) {
		drvb = do_move(drvb, FRONT, n_steps, wall);
	}
	// Serial.println("Here1");
	if(!wall) return drvb;
	drvb = do_move_1_square(drvb, LEFT, wall);

	// Serial.println("Here2");
	if(!wall) return drvb;
	drvb = do_move_1_square(drvb, RIGHT, wall);

	// Serial.println("Here3");
	if(!wall) return drvb;
	drvb = do_move_1_square(drvb, REAR, wall);

	// Serial.println("Here4");
	if(!wall) return drvb;

	// Serial.print("Traceback!");
	int opp_last_move = retrace_last_move();
	if(opp_last_move != -1) {
		// Serial.print(drvb.sq_x);
		// Serial.print(",  ");
		// Serial.print(drvb.sq_y);
		// Serial.print(" :: ");
		// Serial.println(to_string(opp_last_move));
		drvb = move_to_square(drvb, opp_last_move, 1, true);
		set_legality(drvb.sq_x, drvb.sq_y, opposite_move(opp_last_move), Legality::Cannot_go);
		return drvb;
	} 
	// Serial.println(" Failed ");

	fail = true;
	return drvb;
}

Drivebase solve3(Drivebase drvb)
{
	int stored = n_stored_moves();
	if(stored == 0) {
		bool fail = false;
		while(drvb.sq_y != kFieldHeight-1 && !fail) {
			// Serial.println("Start step");
			drvb = step(drvb, fail);
			// Serial.println("End step");
		}
		drvb = forward_dist(drvb, 0.15, kForwardSpeed);
		// if(fail)
			// // Serial.println("T_T");
	} 
	else {
		int n_squares = 1;
		for(int i = 0; i < stored; i+=n_squares) {
			int n_squares = 1;
			while(stored_move(i+n_squares) == stored_move(i)) {
				n_squares++;
			}
			drvb = move_to_square(drvb, stored_move(stored_move(i)), n_squares, true);
		}
	}
	
	delay(500);

	stored = n_stored_moves();
	int n_squares = 1;
	for(int i = stored-1; i >= 0; i-=n_squares) {
		n_squares = 1;
		while(stored_move(i-n_squares) == stored_move(i)) {
			n_squares++;
		}
		drvb = move_to_square(drvb, opposite_move(stored_move(i)), n_squares, true);
	}

	return drvb;
}