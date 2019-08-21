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
    bool operator==(const Direction& other) const
    {
        return this->half == other.half;
    }
};

struct __attribute__((__packed__)) Difference {
    int8_t x;
    int8_t y;
    bool operator==(const Difference& other) const
    {
        return this->x == other.x && this->y == other.y;
    }
};

struct __attribute__((__packed__)) Position {
    uint8_t x;
    uint8_t y;
    bool operator==(const Position& other) const
    {
        return this->x == other.x && this->y == other.y;
    }
    Difference operator-(const Position& other) const
    {
        return { int8_t(this->x - other.x), int8_t(this->y - other.y) };
    }
    Position operator+(const Difference& diff) const
    {
        return { uint8_t(this->x + diff.x), uint8_t(this->y + diff.y) };
    }
};

struct __attribute__((__packed__)) Coordinates {
    Position pos;
    Direction dir;
    bool operator==(const Coordinates& other) const
    {
        return this->pos == other.pos && this->dir == other.dir;
    }
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
    // FIXME: undefined behavior when a < 0 or b < 0
    if (std::numeric_limits<T>::max() - a <= b) {
        return std::numeric_limits<T>::max();
    } else {
        return T(a + b);
    }
}

}
