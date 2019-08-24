#pragma once

#include "mazegraph.h"

namespace Amaze {

/// \~japanese
/// 最短経路ソルバの抽象クラス．
///
/// \tparam TMazeGraph MazeGraphを実装した型
///
/// \~english
/// An abstract class for shortest path solvers.
///
/// \tparam TMazeGraph Class implements MazeGraph
template <typename TMazeGraph>
class Solver {
protected:
    /// \~japanese 迷路のグラフ表現
    /// \~english Graph representation of the maze
    const TMazeGraph& mg;

public:
    Solver(const TMazeGraph& mg)
        : mg(mg)
    {
    }
    virtual ~Solver() {}
    /// \~japanese
    /// 次に訪れるノードの座標を返します．
    /// \returns 次の座標
    ///
    /// \~english
    /// Returns the coordinates of the node the solver is going to visit next.
    /// \returns next coordinates.
    virtual Coordinates getNextCoordinates() const
    {
        return mg.coordByNodeId(getNextNodeId());
    }
    /// \~japanese
    /// 現在のノードの座標を返します．
    /// \returns 現在の座標
    ///
    /// \~english
    /// Returns the coordinates of the current node.
    /// \returns current coordinates.
    virtual Coordinates getCurrentCoordinates() const
    {
        return mg.coordByNodeId(getCurrentNodeId());
    }
    /// \~japanese
    /// 次に訪れるノードのIDを返します．
    /// \returns 次のID
    ///
    /// \~english
    /// Returns the ID of the node the solver is going to visit next.
    /// \returns next ID.
    virtual typename TMazeGraph::NodeId getNextNodeId() const = 0;
    /// \~japanese
    /// 現在のノードのIDを返します．
    /// \returns 現在のID
    ///
    /// \~english
    /// Returns the ID of the current node.
    /// \returns current ID.
    virtual typename TMazeGraph::NodeId getCurrentNodeId() const = 0;

    /// \~japanese
    /// 壁センシング前の処理を行います．
    ///
    /// \~english
    /// Performs pre-sensing routines.
    virtual void preSense() = 0;
    /// \~japanese
    /// 壁センシング後の処理を行います．
    ///
    /// \param [in] sensed_coordinates センシングした壁等の座標
    ///
    /// \~english
    /// Performs post-sensing routines.
    /// \param [in] sensed_coordinates Coordinates of sensed walls etc.
    virtual void postSense(const std::vector<Coordinates>& sensed_coordinates) = 0;

    /// \~japanese
    /// ソルバの内部状態をすべて初期値クリアします．
    ///
    /// \~english
    /// Resets all the internal variables to the initial value.
    virtual void reset() = 0;
    /// \~japanese
    /// ソルバの内部状態を探索開始直前の状態に初期化します．
    ///
    /// \~english
    /// Initializes internal variables to be ready for new search from the start.
    virtual void initialize() = 0;
};

}
