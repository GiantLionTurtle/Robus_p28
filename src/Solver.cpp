
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

void set_legality(bool legal, int x, int y, int move)
{
	int legal_impl = legal ? Legality::Can_go : Legality::Cannot_go;
	switch(move) {
	case LEFT:
			LegalityMatrix[legalityIndex(x, y)] |= legal_impl;
		break;
	case RIGHT:
			set_legality(legal, x+1, y, LEFT);
		break;
	case FRONT:
			set_legality(legal, x, y+1, REAR);
		break;
	case REAR:
			LegalityMatrix[legalityIndex(x, y)] |= legal_impl<<3;
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
}


Drivebase solve(Drivebase drvb)
{
	return drvb;
}

} // !p28