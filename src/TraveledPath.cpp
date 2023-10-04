#include <Arduino.h>
#include "LibRobus.h"
#include "Constants.hpp"
#include "TraveledPath.hpp"

namespace p28{

#define LENGTH 100
static int traveled_path[LENGTH];
static int index = -1;

    void init_path()
    {
        int length = sizeof(traveled_path);
        for(int i=0;i<length;i++)
        {
            traveled_path[i] = -1;
        }
    }

    void add_move(int direction)
    {
        index++;
        traveled_path[index] = direction;
    }

    int get_last_move()
    {
         if(index >= 0)
        {
            return traveled_path[index];
        }
    }

    void delete_last_move()
    {
        if(index >= 0)
        {
            traveled_path[index] = -1;
            index--;
        }
    }

    int retrace_last_move()
    {
        int last_move = get_last_move();
        int go_back;
        switch(last_move)
        {
            case FRONT:
                go_back = REAR;
                break;
            case REAR:
                go_back = FRONT;
                break;
            case LEFT:
                go_back = RIGHT;
                break;
            case RIGHT:
                go_back = LEFT;
                break;
        }
        delete_last_move();
        return go_back;
    }
}