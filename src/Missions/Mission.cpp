
#include "Mission.hpp"

#include <Utils/Vec.hpp>

namespace p28 {
namespace Mission {

bool close_enough(Robot_snapshot const& snapshot_, Robot_target const& target, Epsilon epsilon)
{
    // Weird to compare values of different scales with the same epsilon..
    return  mt::distance2(snapshot_.location.pos, target.location.pos) < epsilon.location2 &&
            mt::distance2(snapshot_.location.front, target.location.front) < epsilon.front2;
}

} // !Mission
} // !p28