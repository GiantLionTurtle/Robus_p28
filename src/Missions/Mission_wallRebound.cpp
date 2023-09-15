
#include "Mission_wallRebound.hpp"

namespace p28 {

namespace Mission {

Pair<Robot_target, WallRebound> WallRebound::update(Robot robot) const
{
    return { Robot_target{}, WallRebound{} };
}

}
} // !p28