
#include "Solver.hpp"
#include "LibRobus.h"

namespace p28 {

// Each char represents the left and bottom "walls" of 
// a case
static char LegalityMatrix[(kFieldWidth+1)*(kFieldHeight+1)];

int legalityIndex(int sq_x, int sq_y)
{
	return sq_x * (kFieldWidth+1) + sq_y;
}

Legality is_legal_move(int sq_x, int sq_y, int move)
{
	switch(move) {
	case LEFT:
		return static_cast<Legality>(LegalityMatrix[legalityIndex(sq_x, sq_y)]);
	case RIGHT:
		return is_legal_move(sq_x+1, sq_y, LEFT);
	case FRONT:
		return is_legal_move(sq_x, sq_y+1, REAR);
	case REAR:
		return static_cast<Legality>(LegalityMatrix[legalityIndex(sq_x, sq_y)]>>3);
	default:
		return Legality::Cannot_go;
	}
}

void set_legality(bool legal, int sq_x, int sq_y, int move)
{
	int legal_impl = legal ? Legality::Can_go : Legality::Cannot_go;
	switch(move) {
	case LEFT:
			LegalityMatrix[legalityIndex(sq_x, sq_y)] = legal_impl;
		break;
	case RIGHT:
			set_legality(legal, sq_x+1, sq_y, LEFT);
		break;
	case FRONT:
			set_legality(legal, sq_x, sq_y+1, REAR);
		break;
	case REAR:
			LegalityMatrix[legalityIndex(sq_x, sq_y)] = legal_impl<<3;
		break;
	default:
		break;
	}
}

void init_legalityMatrix()
{
	for(int i = 0; i < (kFieldWidth+1)*(kFieldHeight+1); ++i) {
		LegalityMatrix[i] = Legality::Unknown;
	}
	// Black walls in the middle
	for(int i = 1; i < 10; i+=2) {
		set_legality(false, 1, i, LEFT);
		set_legality(false, 1, i, RIGHT);
	}
	// Outer walls
	for(int i = 0; i < kFieldWidth+1; ++i) {
		set_legality(false, i, 0, REAR);
		set_legality(false, i, kFieldHeight, REAR);
	}
	for(int i = 0; i < kFieldHeight+1; ++i) {
		set_legality(false, 0, i, LEFT);
		set_legality(false, kFieldWidth, i, LEFT);
	}
}

Drivebase solve(Drivebase drvb)
{}
//    if(is_legal_move(drvb, FRONT) == Legality::Can_go) {                
//     move_to_square(drvb, FRONT, 1);
//    }
//      else if(is_legal_move(drvb, FRONT) == Legality::Cannot_go) {
//        // direction_until_detect(drvb, LEFT, kdetectionDistance, detection);
//      }   
//             if(is_legal_move(drvb, LEFT) == Legality::Can_go){
//             move_to_square(drvb, LEFT, 1);
//             }
//        // else if                                                    //code temporaire 
//        // else if(is_legal_move(drvb, LEFT) == Legality::Can_go){
//        // move_to_square(drvb, FRONT, 1);
//    }
           // else if(is_legal_move(drvb, LEFT) == Legality::Cannot_go){
              //  turn_right(drvb);
                //turn_right(drvb);
            // }
               // else if(is_legal_move(drvb, FRONT) == Legality::Can_go){
                   // move_to_square(drvb, FRONT, 1);
               // }
                   // else {

                  //  }
// }
 // } 
   
   // return drvb;

Drivebase try_move(Drivebase drvb, int move, bool& success)
{
	Legality legal = is_legal_move(drvb.sq_x, drvb.sq_y, move);
	Serial.print("Legal ");
	Serial.print(move);
	Serial.print("  ");
	Serial.println(legal);
	if(legal == Legality::Can_go) {
		drvb = move_to_square(drvb, move, 1);
		success = true;
	} else if(legal == Legality::Unknown) {
		bool wall = false;
		drvb = move_to_square_or_detect(drvb, move, wall);

		set_legality(!wall, drvb.sq_x, drvb.sq_y, move);
		success = !wall;
	} else {
		success = false;
	}
	return drvb;
}
Drivebase step(Drivebase drvb)
{
	bool success = false;
	drvb = try_move(drvb, FRONT, success);
	if(success) return drvb;

	drvb = try_move(drvb, LEFT, success);
	if(success) return drvb;

	drvb = try_move(drvb, RIGHT, success);
	if(success) return drvb;

	// Manage back


	return drvb;
}
Drivebase solve2(Drivebase drvb)
{
	while(drvb.sq_y != 9) {
		drvb = step(drvb);
	}
	return drvb;
}


} // !p28