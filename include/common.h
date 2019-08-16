#pragma once

#include <cstdint>
#include <limits>
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

struct __attribute__((__packed__)) Position {
    uint8_t x;
    uint8_t y;
};

struct __attribute__((__packed__)) Coordinates {
    Position pos;
    Direction dir;
};

static constexpr Direction NoDirection = { 0x0 };
static constexpr Direction Front = { 0x1 };
static constexpr Direction Right = { 0x2 };
static constexpr Direction Back = { 0x4 };
static constexpr Direction Left = { 0x8 };
static constexpr Direction North = { 0x1 };
static constexpr Direction East = { 0x2 };
static constexpr Direction South = { 0x4 };
static constexpr Direction West = { 0x8 };

template <typename T>
T satSum(T a, T b)
{
    if (std::numeric_limits<T>::max() - a <= b) {
        return std::numeric_limits<T>::max();
    } else {
        return a + b;
    }
}

}
