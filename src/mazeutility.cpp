#include "mazeutility.h"

#include <fstream>
#include <iostream>

namespace Amaze {

std::ostream& operator<<(std::ostream& os, Direction d)
{
    if (d.bits.NORTH) {
        os << 'N';
    }
    if (d.bits.EAST) {
        os << 'E';
    }
    if (d.bits.WEST) {
        os << 'W';
    }
    if (d.bits.SOUTH) {
        os << 'S';
    }
    if (d.half == 0) {
        os << '0';
    }
    return os;
}
std::ostream& operator<<(std::ostream& os, Coordinates c)
{
    os << "(" << (int)c.pos.x << ", " << (int)c.pos.y << ", " << c.dir << ")";
    return os;
}

}
