#pragma once

#include "common.h"
#include <bitset>
#include <vector>

namespace Amaze {

/// \~japanese
/// 迷路クラス．
///
/// \tparam W 迷路の幅(区画数)
///
/// \~english
/// A maze class.
///
/// \tparam W Maze width in the number of cells
template <uint8_t W = 32>
class Maze {
private:
    using MazeData = std::bitset<(2 * W - 1) * (2 * W - 1)>;

    /// \~japanese 壁データ
    /// \~english Wall data
    MazeData maze_data;
    /// \~japanese 壁チェックデータ
    /// \~english Checked wall data
    MazeData check_data;
    /// \~japanese スタート位置
    /// \~english Start position
    Position start;
    /// \~japanese ゴール位置
    /// \~english Goal position
    Position goal;

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
    void setInternal(Position p, bool is_checked, bool val)
    {
        // TODO: assert
        if (!((p.x % 2) ^ (p.y % 2)) || p.x >= 2 * W - 1 || p.y >= 2 * W - 1) {
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
    bool isSetInternal(Position p, bool is_checked) const
    {
        // TODO: assert
        if (!((p.x % 2) ^ (p.y % 2)) || p.x >= 2 * W - 1 || p.y >= 2 * W - 1) {
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

public:
    Maze()
        : maze_data(0)
        , check_data(0)
        , start({ 0, 0 })
        , goal({ 0, 0 })
    {
    }

    /// \~japanese
    /// 迷路データをリセットします．
    /// スタート・ゴール位置の情報はリセットしません．
    ///
    /// \~english
    /// Resets maze data.
    /// Does not reset start and goal positions.
    void resetData()
    {
        maze_data.reset();
        check_data.reset();
    }

    /// \~japanese
    /// スタート・ゴール位置を含む迷路データをリセットします．
    ///
    /// \~english
    /// Resets maze data including start and goal positions.
    void resetAll()
    {
        resetData();
        start = { 0, 0 };
        goal = { 0, 0 };
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
    /// スタート位置を返します．
    /// \returns 位置
    ///
    /// \~english
    /// Returns start position.
    /// \returns position.
    Position getStart() const { return start; }
    /// \~japanese
    /// スタート位置を設定します．
    /// \param[in] p 位置
    ///
    /// \~english
    /// Sets start position.
    /// \param[in] p Position
    void setStart(Position p) { start = p; }
    /// \~japanese
    /// ゴール位置を返します．
    /// \returns 位置
    ///
    /// \~english
    /// Returns goal position.
    /// \returns position.
    Position getGoal() const { return goal; }
    /// \~japanese
    /// ゴール位置を設定します．
    /// \param[in] p 位置
    ///
    /// \~english
    /// Sets goal position.
    /// \param[in] p Position
    void setGoal(Position p) { goal = p; }

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
    void toggleWall(Position p)
    {
        setWall(p, !isSetWall(p));
    }
    /// \~japanese
    /// 壁チェックデータの値をトグルします．
    /// \param[in] p 位置
    ///
    /// \~english
    /// Toggles checked wall data.
    /// \param[in] p Position
    void toggleCheckedWall(Position p)
    {
        setCheckedWall(p, !isCheckedWall(p));
    }

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
};

}
