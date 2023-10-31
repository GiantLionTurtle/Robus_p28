#include "LibRobus.h"
#include "IRSensor.hpp"

namespace p28{
    float get_ir_value(int index)
    {
        if (index >=0 && index <= 3)
        {
            float ir = 0;
            //for(int i = 1; i <= 10; i++)
            //{
                ir = ROBUS_ReadIR(index);
            //}
             //ir/=10.0;
             return ir;
        }
        return 0.0;
    }

    float get_distance_ir(int index)
    {
        float ir = get_ir_value(index);
        //y = 9713,2x-1,106
        float mm = 97132*powf(ir, -1.106);
        return mm;
    }
}