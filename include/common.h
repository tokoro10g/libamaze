#pragma once

#include <cstdint>
#include <limits>
#include <utility>

namespace Amaze {

/// \~japanese
/// 方向を表す型．
///
/// \~english
/// Direction.
union Direction {
    uint8_t half : 4;
    struct {
        unsigned north : 1;
        unsigned east : 1;
        unsigned south : 1;
        unsigned west : 1;
    } __attribute__((__packed__)) bits;
    bool operator==(const Direction& other) const
    {
        return this->half == other.half;
    }
} __attribute__((__packed__));

/// \~japanese
/// 迷路内の位置の差分を表す型．
///
/// \~english
/// Position difference in the maze.
struct Difference {
    int8_t x;
    int8_t y;
    bool operator==(const Difference& other) const
    {
        return this->x == other.x && this->y == other.y;
    }
} __attribute__((__packed__));

/// \~japanese
/// 迷路内の位置座標を表す型．
/// 以下の図のように定義された座標系を用いる．
/// \warning 区画数を基準とした座標ではないことに注意する．
///
/// \~english
/// Position coordinates in the maze.
/// The coordinate system is defined as in the following figure.
/// \warning The coordinate system is not based on the cell count.
///
/// \~
/// <pre>
/// |             |             |             |
/// +----(0,3)----+----(2,3)----+----(4,3)----+--
/// |             |             |             |
/// |             |             |             |
/// |    (0,2)  (1,2)  (2,2)  (3,2)  (4,2)  (5,2)
/// |             |             |             |
/// |             |             |             |
/// +----(0,1)----+----(2,1)----+----(4,1)----+--
/// |             |             |             |
/// |             |             |             |
/// |    (0,0)  (1,0)  (2,0)  (3,0)  (4,0)  (5,0)
/// |             |             |             |
/// |             |             |             |
/// +-------------+-------------+-------------+--
/// </pre>
struct Position {
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
} __attribute__((__packed__));

/// \~japanese
/// 迷路内の位置と方向，フラグを含むエージェントの状態を表す型．
///
/// \~english
/// State of the agent including position, direction, and flags in the maze.
struct AgentState {
    Position pos;
    Direction dir;
    uint8_t attribute;
    bool operator==(const AgentState& other) const
    {
        return this->pos == other.pos && this->dir == other.dir && this->attribute == other.attribute;
    }
} __attribute__((__packed__));

static constexpr Direction kNoDirection = { 0x0 };
static constexpr Direction kFront = { 0x1 };
static constexpr Direction kRight = { 0x2 };
static constexpr Direction kBack = { 0x4 };
static constexpr Direction kLeft = { 0x8 };
static constexpr Direction kNorth = { 0x1 };
static constexpr Direction kEast = { 0x2 };
static constexpr Direction kSouth = { 0x4 };
static constexpr Direction kWest = { 0x8 };

static constexpr AgentState kInvalidAgentState = { { uint8_t(-1), uint8_t(-1) }, kNoDirection, 0 };

/// \~japanese
/// 型\p Tの最大値で飽和する和を返します．
/// \tparam T 数値型
/// \param a, b 和を取る数
/// \returns \p a + \p b もしくは \p std::numeric_limits<T>::max()
///
/// \~english
/// Returns sum saturated at the maximum value of the type \p T.
/// \tparam T Numeric type
/// \param a, b Numbers to be added
/// \returns \p a + \p b or \p std::numeric_limits<T>::max()
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
