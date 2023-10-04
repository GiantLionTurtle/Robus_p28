#include <Arduino.h>
#include "LibRobus.h"
#include "Constants.hpp"
#include "TraveledPath.hpp"

#define LENGTH 100
static int traveled_path[LENGTH];
static int index = -1;

void init_path()
{
	for(int i = 0; i < LENGTH-1; i++)
	{
		traveled_path[i] = -1;
	}
}

void add_move(int direction)
{
	index++;
	traveled_path[index] = direction;
}
int stored_move(int ind)
{
	if(ind >= 0 && ind < LENGTH-1) {
		return traveled_path[ind];
	}
	return -1;
}
int get_last_move()
{
	return stored_move(index);
}

void delete_last_move()
{
	if(index >= 0)
	{
		traveled_path[index] = -1;
		index--;
	}
}

int opposite_move(int move)
{
	int opposite;
		switch(move)
	{
		case FRONT:
			opposite = REAR;
			break;
		case REAR:
			opposite = FRONT;
			break;
		case LEFT:
			opposite = RIGHT;
			break;
		case RIGHT:
			opposite = LEFT;
			break;
		default:
			opposite = -1;
			break;
	}
	return opposite;
}
int retrace_last_move()
{
	int last_move = get_last_move();
	int go_back = opposite_move(last_move);
	delete_last_move();
	return go_back;
}

int n_stored_moves()
{
	return index+1;
}