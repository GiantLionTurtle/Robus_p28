

#include "LineDetector.hpp"
#include "LibRobus.h"

namespace p28 {
    static int PIN_IR[8] {A4, A5, A6, A7, A8, A9, A10, A11};
    #define BLACK_TRESH 200
    #define BASIC_TRESH 900


    char get_ir_line()
    {
        char byte_line = 0b00000000;
        for(int i = 0; i < 8; i++)
        {
            bool state = get_ir(i);
             byte_line |= state<<i;
        }
        
        return byte_line;
    }

    bool get_ir(int index)
    {
        bool state = 0;
        if(index >=0 && index <8)
        {
            int irRead = analogRead(PIN_IR[index]);
            if(irRead >= BLACK_TRESH && irRead<=BASIC_TRESH)
            {
                state = 1;
            }
        }
        return state;
    }
}