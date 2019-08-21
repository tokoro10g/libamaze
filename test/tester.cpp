#include "dstarlite.h"
#include "maze.h"
#include "mazegraph.h"
#include "mazeutility.h"
#include <bitset>
#include <iostream>
#include <sstream>

using namespace Amaze;

template <uint8_t W>
void senseFourWay(Maze<W>& maze, const Maze<W>& reference_maze, Coordinates c, std::vector<Coordinates>& changed_coordinates)
{
    constexpr int8_t dx[4] = { 0, 1, 0, -1 };
    constexpr int8_t dy[4] = { 1, 0, -1, 0 };
    for (int i = 0; i < 4; i++) {
        Position p = { uint8_t(c.pos.x + dx[i]), uint8_t(c.pos.y + dy[i]) };
        if (p.x > 2 * W || p.y > 2 * W) {
            continue;
        }
        if (maze.isCheckedWall(p)) {
            continue;
        }
        maze.setCheckedWall(p, true);
        if (reference_maze.isSetWall(p)) {
            maze.setWall(p, true);
            Position pto = { uint8_t(p.x + dx[i]), uint8_t(p.y + dy[i]) };
            changed_coordinates.push_back({ pto, NoDirection });
        } else {
            maze.setWall(p, false);
        }
    }
}

template <uint8_t W>
void senseSixWay(Maze<W>& maze, const Maze<W>& reference_maze, Coordinates c, std::vector<Coordinates>& changed_coordinates)
{
    constexpr int8_t dx[8] = { 0, 1, 2, 1, 0, -1, -2, -1 };
    constexpr int8_t dy[8] = { 2, 1, 0, -1, -2, -1, 0, 1 };
    for (int i = 0; i < 8; i++) {
        if (c.pos.x % 2 == 0 && c.pos.y % 2 == 1 && (i == 2 || i == 6)) {
            continue;
        } else if (c.pos.x % 2 == 1 && c.pos.y % 2 == 0 && (i == 0 || i == 4)) {
            continue;
        }
        Position p = { uint8_t(c.pos.x + dx[i]), uint8_t(c.pos.y + dy[i]) };
        if (p.x > 2 * W || p.y > 2 * W) {
            continue;
        }
        if (maze.isCheckedWall(p)) {
            continue;
        }
        maze.setCheckedWall(p, true);
        if (reference_maze.isSetWall(p)) {
            maze.setWall(p, true);
            changed_coordinates.push_back({ p, NoDirection });
        } else {
            maze.setWall(p, false);
        }
    }
}

int main()
{
    const std::string maze_data = "1\n"
                                  "32\n"
                                  "32\n"
                                  "19\n"
                                  "20\n"
                                  "9 3 9 3 9 5 5 5 3 9 1 3 9 5 5 5 5 5 3 9 1 3 9 5 5 5 5 5 3 9 1 3\n"
                                  "c 6 a a c 5 5 3 a c 4 6 c 5 5 5 5 3 a c 4 6 c 5 5 5 5 3 a c 4 6\n"
                                  "9 5 6 a 9 5 5 6 a d 1 5 1 5 5 5 3 a c 5 5 5 5 5 5 5 5 6 c 5 5 3\n"
                                  "c 1 7 a c 5 5 3 8 3 c 3 e 9 5 5 6 c 5 5 5 5 5 1 5 5 5 3 9 3 9 2\n"
                                  "9 4 3 a 9 1 3 a a c 3 c 3 a 9 1 3 9 5 5 5 5 3 a 9 1 3 a a c 6 e\n"
                                  "c 1 6 a 8 0 2 a 8 3 c 3 a a 8 0 2 a 9 5 5 5 6 a 8 0 2 a c 5 5 3\n"
                                  "9 4 3 a c 4 6 a a c 3 c 2 a c 4 6 a c 5 5 5 3 a c 4 6 a b 9 3 a\n"
                                  "c 1 6 c 1 5 7 a c 5 4 5 2 a d 5 5 2 9 5 5 5 6 c 5 5 1 6 8 6 c 6\n"
                                  "d 4 3 9 4 5 5 2 9 5 5 5 2 a d 5 5 2 8 5 5 5 3 9 5 1 4 3 8 5 5 7\n"
                                  "9 3 a a 9 3 9 2 a 9 1 3 a a 9 3 b a a 9 1 3 a c 1 4 1 6 a 9 1 3\n"
                                  "8 2 a 8 2 8 2 a a 8 0 2 a a a a a a a 8 0 2 a 9 4 1 4 3 a 8 0 2\n"
                                  "c 6 a a 8 2 8 2 a c 4 6 a a a a a a a 8 4 6 a c 1 4 1 6 a c 4 6\n"
                                  "9 5 6 8 6 c 6 e 8 5 5 5 6 8 6 c 6 a c 4 5 5 4 5 4 5 4 7 c 5 5 3\n"
                                  "c 1 7 8 5 5 5 3 8 5 5 5 7 c 5 5 5 6 d 1 5 1 5 1 5 5 5 3 9 5 1 6\n"
                                  "d 0 7 a 9 1 3 a c 5 5 5 3 b 9 1 3 b d 0 5 0 7 a 9 1 3 a c 1 4 3\n"
                                  "d 0 7 a 8 0 2 a 9 5 5 5 6 a 8 0 2 a 9 4 1 4 3 a 8 0 2 a 9 4 1 6\n"
                                  "d 0 7 a c 4 6 a c 5 5 5 3 a c 4 6 a 8 5 0 5 2 a c 4 6 8 4 1 4 3\n"
                                  "9 4 7 8 5 5 5 6 9 5 5 5 4 4 5 5 5 4 4 5 4 5 6 c 5 5 1 6 9 4 1 6\n"
                                  "c 5 3 8 5 5 5 7 c 5 5 5 3 9 1 5 1 1 1 5 5 5 3 9 3 9 2 b 8 5 4 7\n"
                                  "9 3 a c 5 5 5 3 b 9 1 3 a a e b e a a 9 1 3 8 2 8 2 8 2 a 9 1 3\n"
                                  "8 2 a 9 5 5 5 6 a 8 0 2 a 8 1 4 1 2 a 8 0 2 a 8 2 8 2 a a 8 0 2\n"
                                  "c 6 a c 5 5 5 3 a c 4 6 a a e b e a a c 4 6 8 2 8 2 8 2 a c 4 6\n"
                                  "9 5 6 9 5 5 1 6 8 5 5 5 6 c 1 4 5 6 c 5 5 1 6 8 6 c 6 c 4 5 5 3\n"
                                  "c 5 3 c 5 5 4 3 a 9 3 9 3 9 4 5 5 3 9 5 1 4 3 8 5 5 5 1 1 1 1 2\n"
                                  "9 3 a b 9 1 3 a 8 2 8 2 8 2 9 1 3 a c 1 4 1 6 a 9 1 3 a a a a a\n"
                                  "a a a a 8 0 2 a a 8 2 8 2 a 8 0 2 a 9 4 1 4 3 a 8 0 2 a a a a a\n"
                                  "a c 6 a c 4 6 a 8 2 8 2 a a c 4 6 8 4 1 4 1 6 a c 4 6 a a a a a\n"
                                  "8 1 7 8 5 5 5 2 e c 2 c 6 c 5 5 1 6 d 4 5 4 5 4 5 5 5 6 c 4 4 2\n"
                                  "a c 3 c 5 5 3 a 9 5 4 5 3 9 3 9 2 b 9 5 5 5 1 1 5 1 5 3 b b b a\n"
                                  "a b a 9 5 5 6 a a 9 1 3 a a 8 2 8 2 a 9 1 3 a a 9 2 9 0 0 0 0 2\n"
                                  "a a a c 5 5 3 a a 8 0 2 a 8 2 8 2 a a 8 0 2 a 8 6 8 6 a a e e a\n"
                                  "e e c 5 5 5 6 c 6 c 4 6 c 6 c 6 c 4 6 c 4 6 e c 5 4 5 4 4 5 5 6\n";
    std::istringstream iss(maze_data);

    constexpr uint8_t max_maze_width = 32;
    Maze<max_maze_width> reference_maze;
    Utility::loadMazeFromStream(reference_maze, iss);

    Utility::printMaze(reference_maze);
    std::cout << std::endl;

    //////////////////////////////////////////////////////////////////////////////////////////////

    FourWayStepMapGraph mg1(reference_maze);
    auto solver1 = DStarLite(mg1);
    auto e11 = mg1.getEdge({ { 0, 2 }, NoDirection }, { { 0, 2 }, NoDirection });
    auto e12 = mg1.getEdge({ { 0, 0 }, NoDirection }, { { 0, 2 }, NoDirection });
    auto e13 = mg1.getEdge({ { 0, 0 }, NoDirection }, { { 2, 0 }, NoDirection });
    std::cout << (e11.first ? "true" : "false") << " " << (int)e11.second << std::endl;
    std::cout << (e12.first ? "true" : "false") << " " << (int)e12.second << std::endl;
    std::cout << (e13.first ? "true" : "false") << " " << (int)e13.second << std::endl;
    std::cout << std::endl;
    std::cout << mg1.size << std::endl;
    std::cout << std::endl;

    FourWayStepMapGraph<uint16_t> mg2(reference_maze);
    auto e21 = mg2.getEdge({ { 0, 2 }, NoDirection }, { { 0, 2 }, NoDirection });
    auto e22 = mg2.getEdge({ { 0, 0 }, NoDirection }, { { 0, 2 }, NoDirection });
    auto e23 = mg2.getEdge({ { 0, 0 }, NoDirection }, { { 2, 0 }, NoDirection });
    std::cout << (e21.first ? "true" : "false") << " " << (int)e21.second << std::endl;
    std::cout << (e22.first ? "true" : "false") << " " << (int)e22.second << std::endl;
    std::cout << (e23.first ? "true" : "false") << " " << (int)e23.second << std::endl;
    std::cout << std::endl;

    FourWayStepMapGraph<float> mg3(reference_maze);
    auto e31 = mg3.getEdge({ { 0, 2 }, NoDirection }, { { 0, 2 }, NoDirection });
    auto e32 = mg3.getEdge({ { 0, 0 }, NoDirection }, { { 0, 2 }, NoDirection });
    auto e33 = mg3.getEdge({ { 0, 0 }, NoDirection }, { { 2, 0 }, NoDirection });
    std::cout << (e31.first ? "true" : "false") << " " << e31.second << std::endl;
    std::cout << (e32.first ? "true" : "false") << " " << e32.second << std::endl;
    std::cout << (e33.first ? "true" : "false") << " " << e33.second << std::endl;
    std::cout << std::endl;

    SixWayWallNodeGraph mg4(reference_maze);
    auto solver4 = DStarLite(mg4);
    auto e41 = mg4.getEdge({ { 0, 3 }, South }, { { 1, 2 }, East });
    auto e42 = mg4.getEdge({ { 0, 3 }, North }, { { 0, 5 }, North });
    auto e43 = mg4.getEdge({ { 1, 2 }, East }, { { 2, 3 }, North });
    std::cout << (e41.first ? "true" : "false") << " " << (int)e41.second << std::endl;
    std::cout << (e42.first ? "true" : "false") << " " << (int)e42.second << std::endl;
    std::cout << (e43.first ? "true" : "false") << " " << (int)e43.second << std::endl;
    std::cout << std::endl;
    std::cout << mg4.size << std::endl;
    std::cout << std::endl;

    //////////////////////////////////////////////////////////////////////////////////////////////

    std::cout << std::endl;
    std::cout << "=======Testing D* Lite for 4-way Graph=======" << std::endl;
    std::cout << std::endl;

    solver1.initialize();
    decltype(mg1)::NodeId id_goal = mg1.getGoalNodeId();
    std::cout << "The start is " << mg1.getStartNodeId() << std::endl;
    std::cout << "The goal is " << id_goal << std::endl;
    std::cout << std::endl;

    while (solver1.getCurrentNodeId() != id_goal) {
        std::cout << mg1.coordByNodeId(solver1.getCurrentNodeId()) << ": " << (int)solver1.getCurrentNodeId() << std::endl;
        solver1.preSense();
        solver1.postSense(std::vector<Coordinates>());
    }
    std::cout << id_goal << std::endl;

    //////////////////////////////////////////////////////////////////////////////////////////////

    std::cout << std::endl;
    std::cout << "=======Testing D* Lite for 6-way Graph=======" << std::endl;
    std::cout << std::endl;

    solver4.initialize();
    decltype(mg4)::NodeId id_goal4 = mg4.getGoalNodeId();
    std::cout << "The start is " << mg4.getStartNodeId() << std::endl;
    std::cout << "The goal is " << id_goal4 << std::endl;
    std::cout << std::endl;

    while (solver4.getCurrentNodeId() != id_goal4) {
        std::cout << mg4.coordByNodeId(solver4.getCurrentNodeId()) << ": " << (int)solver4.getCurrentNodeId() << std::endl;
        solver4.preSense();
        solver4.postSense(std::vector<Coordinates>());
    }
    std::cout << id_goal4 << std::endl;

    //////////////////////////////////////////////////////////////////////////////////////////////

    std::cout << std::endl;
    std::cout << "=======Testing Dynamic Search Using D* Lite for 4-way Graph=======" << std::endl;
    std::cout << std::endl;

    Maze<max_maze_width> maze;
    Position goal_position = reference_maze.getGoal();
    Utility::loadEmptyMaze(goal_position.x, goal_position.y, maze);
    maze.setWall({ 1, 0 }, true);
    maze.setCheckedWall({ 0, 1 }, true);
    maze.setCheckedWall({ 1, 0 }, true);
    FourWayStepMapGraph mg5(maze);
    auto solver5 = DStarLite(mg5);

    solver5.initialize();
    Utility::printMaze(maze);
    decltype(mg5)::NodeId id_goal5 = mg5.getGoalNodeId();
    std::cout << "The start is " << mg5.getStartNodeId() << std::endl;
    std::cout << "The goal is " << id_goal5 << std::endl;
    std::cout << std::endl;

    //for(int i=0;i<80;i++){
    while (solver5.getCurrentNodeId() != id_goal5) {
        //Utility::printMaze(maze);
        uint16_t id = solver5.getCurrentNodeId();
        //CellData cd = maze.getCell(id);
        //std::cout << id << " " << mg5.coordByNodeId(id) << ": " << std::bitset<8>(cd.byte).to_string() << std::endl;
        std::cout << mg5.coordByNodeId(id) << std::endl;
        std::vector<Coordinates> changed_coordinates;

        solver5.preSense();
        id = solver5.getCurrentNodeId();
        senseFourWay(maze, reference_maze, mg5.coordByNodeId(id), changed_coordinates);
        //std::cout << "After sense: " << std::endl;
        //cd = maze.getCell(id);
        //std::cout << id << " " << mg5.coordByNodeId(id) << ": " << std::bitset<8>(cd.byte).to_string() << std::endl;
        //std::cout << "Changed coordinates: ";
        //for (auto c : changed_coordinates) {
        //    std::cout << c << ", ";
        //}
        //std::cout << std::endl;
        solver5.postSense(changed_coordinates);
    }
    std::cout << id_goal5 << std::endl;

    //////////////////////////////////////////////////////////////////////////////////////////////

    std::cout << std::endl;
    std::cout << "=======Testing Dynamic Search Using D* Lite for 6-way Graph=======" << std::endl;
    std::cout << std::endl;

    Utility::loadEmptyMaze(goal_position.x, goal_position.y, maze);
    maze.setWall({ 1, 0 }, true);
    maze.setCheckedWall({ 0, 1 }, true);
    maze.setCheckedWall({ 1, 0 }, true);
    SixWayWallNodeGraph mg6(maze);
    auto solver6 = DStarLite(mg6);

    solver6.initialize();
    Utility::printMaze(maze);
    decltype(mg6)::NodeId id_goal6 = mg6.getGoalNodeId();
    std::cout << "The start is " << mg6.getStartNodeId() << std::endl;
    std::cout << "The goal is " << id_goal6 << std::endl;
    std::cout << std::endl;

    std::vector<Coordinates> temp1;
    senseSixWay(maze, reference_maze, mg6.coordByNodeId(mg6.getStartNodeId()), temp1);

    //for(int i=0;i<80;i++){
    while (solver6.getCurrentNodeId() != id_goal6) {
        //Utility::printMaze(maze);
        uint16_t id = solver6.getCurrentNodeId();
        //CellData cd = maze.getCell(id);
        //std::cout << id << " " << mg5.coordByNodeId(id) << ": " << std::bitset<8>(cd.byte).to_string() << std::endl;
        std::cout << mg6.coordByNodeId(id) << std::endl;
        std::vector<Coordinates> changed_coordinates;

        solver6.preSense();
        id = solver6.getCurrentNodeId();
        senseSixWay(maze, reference_maze, mg6.coordByNodeId(id), changed_coordinates);
        //std::cout << "After sense: " << std::endl;
        //cd = maze.getCell(id);
        //std::cout << id << " " << mg5.coordByNodeId(id) << ": " << std::bitset<8>(cd.byte).to_string() << std::endl;
        //std::cout << "Changed coordinates: ";
        //for (auto c : changed_coordinates) {
        //    std::cout << c << ", ";
        //}
        //std::cout << std::endl;
        solver6.postSense(changed_coordinates);
    }
    std::cout << id_goal6 << std::endl;
    return 0;
}
