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

static constexpr Direction DirFront = { 0x1 };
static constexpr Direction DirRight = { 0x2 };
static constexpr Direction DirBack = { 0x4 };
static constexpr Direction DirLeft = { 0x8 };
static constexpr Direction DirNorth = { 0x1 };
static constexpr Direction DirEast = { 0x2 };
static constexpr Direction DirSouth = { 0x4 };
static constexpr Direction DirWest = { 0x8 };

}
