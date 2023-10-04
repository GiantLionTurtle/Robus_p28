
#include "Solver.hpp"
#include "LibRobus.h"


#define LEFT_WALL_BIT_MASK (1 | 2 | 4)
#define REAR_WALL_BIT_MASK (8 | 16 | 32)
#define REAR_WALL_BIT_OFFSET 3

namespace p28 {


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
	int legal = is_move_legal(drvb.sq_x, drvb.sq_y, move);
	// Serial.print("Legal ");
	// Serial.print(move);
	// Serial.print("  ");
	// Serial.println(legal);
	if(legal == Legality::Can_go) {
		drvb = move_to_square(drvb, move, 1);
		success = true;
		delay(kDecelerationDelay);
	} else if(legal == Legality::Unknown) {
		bool wall = false;
		int init_sq_x = drvb.sq_x;
		int init_sq_y = drvb.sq_y;

		drvb = move_to_square_or_detect(drvb, move, wall);
		
		set_legality(init_sq_x, init_sq_y, move, wall ? Legality::Cannot_go : Legality::Can_go);
		success = !wall;
		delay(kDecelerationDelay);
	} else {
		success = false;
	}
	return drvb;
}
Drivebase step(Drivebase drvb)
{
	bool success = false;
	Serial.println("Try front");
	drvb = try_move(drvb, FRONT, success);
	if(success) return drvb;

	Serial.println("Try left");
	drvb = try_move(drvb, LEFT, success);
	if(success) return drvb;

	Serial.println("Try right");
	drvb = try_move(drvb, RIGHT, success);
	if(success) return drvb;

	// Manage back

	return drvb;
}
Drivebase solve2(Drivebase drvb)
{
	while(drvb.sq_y != 9) {
		Serial.print("square: ");
		Serial.print(drvb.x);
		Serial.print(",  ");
		Serial.println(drvb.sq_x);

		drvb = step(drvb);
	}
	return drvb;
}


} // !p28