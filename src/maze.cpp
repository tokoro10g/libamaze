#include "maze.h"

namespace Amaze {
    void Maze::setWallInternal(const Coord& c, Direction dir, bool is_checked, bool val)
    {
        data[c.y * w + c.x].byte |= (dir.half << (4 * is_checked));

        if (dir.bits.SOUTH && c.y != 0) {
            data[(c.y - 1) * w + c.x].bits.NORTH = val;
        }
        if (dir.bits.EAST && c.x != w - 1) {
            data[c.y * w + c.x + 1].bits.WEST = val;
        }
        if (dir.bits.NORTH && c.y != w - 1) {
            data[(c.y + 1) * w + c.x].bits.SOUTH = val;
        }
        if (dir.bits.WEST && c.x != 0) {
            data[c.y * w + c.x - 1].bits.EAST = val;
        }
    }
    void Maze::resize(uint8_t _w, uint8_t _h){
        w = _w;
        h = _h;
        data.resize(w * h);
    }
}
