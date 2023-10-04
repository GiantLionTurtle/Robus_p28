
#include "Solver.hpp"
#include "LibRobus.h"

namespace p28 {

Legality is_legal_move(Drivebase drvb, int move)
{
    return Can_go;
}

void set_legality(bool legal, int x, int y, int move)
{
    
}

Drivebase solve(Drivebase drvb)
{
   if(is_legal_move(drvb, FRONT) == Legality::Can_go) {                
    move_to_square(drvb, FRONT, 1);
   }
     else if(is_legal_move(drvb, FRONT) == Legality::Cannot_go) {
       // direction_until_detect(drvb, LEFT, kdetectionDistance, detection);
     }   
            if(is_legal_move(drvb, LEFT) == Legality::Can_go){
            move_to_square(drvb, LEFT, 1);
            }
       // else if                                                    //code temporaire 
       // else if(is_legal_move(drvb, LEFT) == Legality::Can_go){
       // move_to_square(drvb, FRONT, 1);
   }
           // else if(is_legal_move(drvb, LEFT) == Legality::Cannot_go){
              //  turn_right(drvb);
                //turn_right(drvb);
            }
               // else if(is_legal_move(drvb, FRONT) == Legality::Can_go){
                   // move_to_square(drvb, FRONT, 1);
               // }
                   // else {

                  //  }
// }
 // } 
   
   // return drvb;




//} // !p28