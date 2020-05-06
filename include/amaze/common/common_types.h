/**
 * Copyright (c) 2020 Tokoro
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#ifndef AMAZE_COMMON_COMMON_TYPES_H_
#define AMAZE_COMMON_COMMON_TYPES_H_

#include <bitset>
#include <cstdint>
#include <limits>
#include <ostream>
#include <type_traits>
#include <unordered_set>
#include <utility>
#include <vector>

namespace amaze {

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
  bool operator==(const Direction &other) const {
    return this->half == other.half;
  }
} __attribute__((__packed__));

/// \~japanese
/// 位置の種類を表す列挙型．
///
/// \~english
/// Enumerator that represents types of positions.
enum class PositionType { kCell = 0, kWall, kPillar };

/// \~japanese
/// 迷路内の位置の差分を表す型．
///
/// \~english
/// Position difference in the maze.
struct Difference {
  int8_t x;
  int8_t y;
  bool operator==(const Difference &other) const {
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
/// \warning Note that the coordinate system is not based on the cell count.
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
  bool operator==(const Position &other) const {
    return this->x == other.x && this->y == other.y;
  }
  bool operator!=(const Position &other) const { return !(*this == other); }
  Difference operator-(const Position &other) const {
    return {int8_t(this->x - other.x), int8_t(this->y - other.y)};
  }
  Position operator+(const Difference &diff) const {
    return {uint8_t(this->x + diff.x), uint8_t(this->y + diff.y)};
  }
  PositionType type() const {
    if (this->x % 2 == 0 && this->y % 2 == 0) {
      return PositionType::kCell;
    } else if (this->x % 2 == 1 && this->y % 2 == 1) {
      return PositionType::kPillar;
    } else {
      return PositionType::kWall;
    }
  }
} __attribute__((__packed__));

/// \~japanese
/// 迷路内の位置と方向，ユーザ定義のフラグを含むエージェントの状態を表す型．
///
/// \~english
/// State of the agent including position, direction, and user-defined flags in
/// the maze.
struct AgentState {
  Position pos;
  Direction dir;
  uint8_t attribute;
  bool operator==(const AgentState &other) const {
    return this->pos == other.pos && this->dir == other.dir &&
           this->attribute == other.attribute;
  }
} __attribute__((__packed__));

/* */

static constexpr Direction kNoDirection = {0x0};
static constexpr Direction kFront = {0x1};
static constexpr Direction kRight = {0x2};
static constexpr Direction kBack = {0x4};
static constexpr Direction kLeft = {0x8};
static constexpr Direction kNorth = {0x1};
static constexpr Direction kEast = {0x2};
static constexpr Direction kSouth = {0x4};
static constexpr Direction kWest = {0x8};

static constexpr AgentState kInvalidAgentState = {
    {uint8_t(-1), uint8_t(-1)}, kNoDirection, 0};
static constexpr Position kInvalidPosition = {uint8_t(-1), uint8_t(-1)};

static constexpr uint8_t kDefaultMazeWidth = 32;
}  // namespace amaze

namespace std {

template <>
struct hash<amaze::Direction> {
  std::size_t operator()(amaze::Direction const &dir) const noexcept {
    return dir.half;
  }
  static constexpr size_t hash_max = 15;
};

template <>
struct hash<amaze::Difference> {
  std::size_t operator()(amaze::Difference const &diff) const noexcept {
    return static_cast<uint8_t>(diff.y) * 256 + static_cast<uint8_t>(diff.x);
  }
  static constexpr size_t hash_max = 65535;
};

template <>
struct hash<amaze::Position> {
  std::size_t operator()(amaze::Position const &pos) const noexcept {
    return pos.y * 256 + pos.x;
  }
  static constexpr size_t hash_max = 65535;
};

template <>
struct hash<amaze::AgentState> {
  std::size_t operator()(amaze::AgentState const &as) const noexcept {
    return hash<amaze::Position>()(as.pos) +
           hash<amaze::Direction>()(as.dir) *
               (hash<amaze::Position>::hash_max + 1) +
           as.attribute * (hash<amaze::Position>::hash_max + 1) *
               (hash<amaze::Direction>::hash_max + 1);
  }
};

}  // namespace std

namespace amaze {

/// \~japanese
/// 迷路クラス．
///
/// \tparam W 迷路の幅(区画数)
///
/// \~english
/// A maze class.
///
/// \tparam W Maze width in the number of cells
template <uint8_t W = kDefaultMazeWidth>
class Maze {
  static_assert(W <= 128);

 public:
  /// \~japanese スタート位置
  /// \~english Start position
  Position start;
  /// \~japanese ゴール位置
  /// \~english Goal positions
  std::unordered_set<Position> goals;
  /// \~japanese 迷路の最大幅
  /// \~english Maximum width of the maze
  static constexpr uint8_t kMaxWidth = W;

  Maze() : start({0, 0}), goals(), maze_data(0), check_data(0) {}

  /// \~japanese
  /// 迷路データをリセットします．
  /// スタート・ゴール位置の情報はリセットしません．
  ///
  /// \~english
  /// Resets maze data.
  /// Does not reset start and goal positions.
  void resetData() {
    maze_data.reset();
    check_data.reset();
  }

  /// \~japanese
  /// スタート・ゴール位置を含む迷路データをリセットします．
  ///
  /// \~english
  /// Resets maze data including start and goal positions.
  void resetAll() {
    resetData();
    start = {0, 0};
    goals.clear();
  }

  /// \~japanese
  /// 迷路の幅を返します．
  /// \returns 区画数
  ///
  /// \~english
  /// Returns the width.
  /// \returns the number of cells.
  uint8_t getWidth() const { return W; }

  /// \~japanese
  /// 壁データの値を設定します．
  /// \param[in] p 位置
  /// \param[in] is_set 壁が存在するとき \p true
  ///
  /// \~english
  /// Sets wall data.
  /// \param[in] p Position
  /// \param[in] is_set \p true if the wall exists
  void setWall(Position p, bool is_set) { setInternal(p, false, is_set); }
  /// \~japanese
  /// 壁チェックデータの値を設定します．
  /// \param[in] p 位置
  /// \param[in] is_set チェック済みのとき \p true
  ///
  /// \~english
  /// Sets checked wall data.
  /// \param[in] p Position
  /// \param[in] is_set \p true if the wall is checked
  void setCheckedWall(Position p, bool is_set) { setInternal(p, true, is_set); }
  /// \~japanese
  /// 壁データの値をトグルします．
  /// \param[in] p 位置
  ///
  /// \~english
  /// Toggles wall data.
  /// \param[in] p Position
  void toggleWall(Position p) { setWall(p, !isSetWall(p)); }
  /// \~japanese
  /// 壁チェックデータの値をトグルします．
  /// \param[in] p 位置
  ///
  /// \~english
  /// Toggles checked wall data.
  /// \param[in] p Position
  void toggleCheckedWall(Position p) { setCheckedWall(p, !isCheckedWall(p)); }

  /// \~japanese
  /// 壁が存在するかどうかを返します．
  /// \param[in] p 位置
  /// \returns 壁が存在するとき \p true
  ///
  /// \~english
  /// Returns whether the wall exists.
  /// \param[in] p Position
  /// \returns \p true if the wall exists.
  bool isSetWall(Position p) const { return isSetInternal(p, false); }
  /// \~japanese
  /// 壁がチェック済みかどうかを返します．
  /// \param[in] p 位置
  /// \returns 壁がチェック済みのとき \p true
  ///
  /// \~english
  /// Returns whether the wall is checked.
  ///
  /// \param[in] p Position
  /// \returns \p true if the wall is checked.
  bool isCheckedWall(Position p) const { return isSetInternal(p, true); }

 private:
  using MazeData = std::bitset<(2 * W - 1) * (2 * W - 1)>;

  /// \~japanese 壁データ
  /// \~english Wall data
  MazeData maze_data;
  /// \~japanese 壁チェックデータ
  /// \~english Checked wall data
  MazeData check_data;

  /// \~japanese
  /// 迷路データを操作する内部関数．
  ///
  /// \param[in] p 位置
  /// \param[in] is_checked 壁チェックデータを操作するとき \p true
  /// \param[in] val 変更後の値
  ///
  /// \~english
  /// Internal function which manipulates maze data.
  ///
  /// \param[in] p Position
  /// \param[in] is_checked \p true if modifying wall check data
  /// \param[in] val New value
  void setInternal(Position p, bool is_checked, bool val) {
    if (p.type() != PositionType::kWall || p.x >= 2 * W - 1 ||
        p.y >= 2 * W - 1) /* [[unlikely]] */ {
      // not a wall
      return;
    }

    uint16_t idx = uint16_t(p.y * (2 * W - 1) + p.x);
    if (is_checked) {
      check_data[idx] = val;
    } else {
      maze_data[idx] = val;
    }
  }

  /// \~japanese
  /// 迷路データを得る内部関数．
  ///
  /// \param[in] p 位置
  /// \param[in] is_checked 壁チェックデータを得るとき \p true
  ///
  /// \~english
  /// Internal function which queries maze data.
  ///
  /// \param[in] p Position
  /// \param[in] is_checked \p true if querying wall check data
  bool isSetInternal(Position p, bool is_checked) const {
    if (p.type() != PositionType::kWall || p.x >= 2 * W - 1 ||
        p.y >= 2 * W - 1) /* [[unlikely]] */ {
      // not a wall
      return false;
    }

    uint16_t idx = uint16_t(p.y * (2 * W - 1) + p.x);
    if (is_checked) {
      return check_data[idx];
    } else {
      return maze_data[idx];
    }
  }
};

}  // namespace amaze

#endif  // AMAZE_COMMON_COMMON_TYPES_H_
