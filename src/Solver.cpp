
#include "Solver.hpp"
#include "LibRobus.h"

namespace p28 {

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
			LegalityMatrix[legalityIndex(sq_x, sq_y)] |= legal_impl;
		break;
	case RIGHT:
			set_legality(legal, sq_x+1, sq_y, LEFT);
		break;
	case FRONT:
			set_legality(legal, sq_x, sq_y+1, REAR);
		break;
	case REAR:
			LegalityMatrix[legalityIndex(sq_x, sq_y)] |= legal_impl<<3;
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
{
	return drvb;
}

} // !p28