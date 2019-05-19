#pragma once

#include <cstdint>
#include <utility>

namespace Amaze {

union __attribute__((__packed__)) Direction {
    uint8_t half : 4;
    struct __attribute__((__packed__)) {
        unsigned NORTH : 1;
        unsigned EAST : 1;
        unsigned SOUTH : 1;
        unsigned WEST : 1;
    } bits;
};

struct __attribute__((__packed__)) Coord {
    uint8_t x;
    uint8_t y;
    Direction dir;
};

}
