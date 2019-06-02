#include "dstarlite.h"
#include "maze.h"
#include "mazegraph.h"
#include "mazeutility.h"
#include <bitset>
#include <fstream>
#include <iostream>

using namespace Amaze;

void sense(Maze& maze, const Maze& reference_maze, Coordinates c, std::vector<Coordinates>& changed_coordinates)
{
    c.dir = North;
    for (int i = 0; i < 4; i++) {
        c.dir.half = (1 << i);
        if (maze.isCheckedWall(c)) {
            continue;
        }
        maze.setCheckedWall(c);
        if (reference_maze.isSetWall(c)) {
            maze.setWall(c);
            changed_coordinates.push_back(c);
        } else {
            maze.resetWall(c);
        }
    }
}

int main()
{
    constexpr uint8_t max_maze_width = 32;
    Maze reference_maze(max_maze_width, max_maze_width);
    Utility::loadMazeFromFile(reference_maze, "../micromouse_mazedat/maze2017halfexp.dat");

    Utility::printMaze(reference_maze);
    std::cout << std::endl;

    FourWayStepMapGraph mg1(reference_maze);
    auto solver1 = DStarLite(mg1);
    auto e11 = mg1.getEdge({ 0, 1, North }, { 0, 1, East });
    auto e12 = mg1.getEdge({ 0, 1, North }, { 0, 2, North });
    auto e13 = mg1.getEdge({ 0, 1, East }, { 1, 2, North });
    std::cout << (e11.first ? "true" : "false") << " " << (int)e11.second << std::endl;
    std::cout << (e12.first ? "true" : "false") << " " << (int)e12.second << std::endl;
    std::cout << (e13.first ? "true" : "false") << " " << (int)e13.second << std::endl;
    std::cout << std::endl;
    std::cout << mg1.size << std::endl;
    std::cout << std::endl;

    FourWayStepMapGraph<uint16_t> mg2(reference_maze);
    auto e21 = mg2.getEdge({ 0, 1, North }, { 0, 1, East });
    auto e22 = mg2.getEdge({ 0, 1, North }, { 0, 2, North });
    auto e23 = mg2.getEdge({ 0, 1, East }, { 1, 2, North });
    std::cout << (e21.first ? "true" : "false") << " " << (int)e21.second << std::endl;
    std::cout << (e22.first ? "true" : "false") << " " << (int)e22.second << std::endl;
    std::cout << (e23.first ? "true" : "false") << " " << (int)e23.second << std::endl;
    std::cout << std::endl;

    FourWayStepMapGraph<float> mg3(reference_maze);
    auto e31 = mg3.getEdge({ 0, 1, North }, { 0, 1, East });
    auto e32 = mg3.getEdge({ 0, 1, North }, { 0, 2, North });
    auto e33 = mg3.getEdge({ 0, 1, East }, { 1, 2, North });
    std::cout << (e31.first ? "true" : "false") << " " << e31.second << std::endl;
    std::cout << (e32.first ? "true" : "false") << " " << e32.second << std::endl;
    std::cout << (e33.first ? "true" : "false") << " " << e33.second << std::endl;
    std::cout << std::endl;

    SixWayWallNodeGraph mg4(reference_maze);
    auto solver4 = DStarLite(mg4);
    auto e41 = mg4.getEdge({ 0, 1, North }, { 0, 1, East });
    auto e42 = mg4.getEdge({ 0, 1, North }, { 0, 2, North });
    auto e43 = mg4.getEdge({ 0, 1, East }, { 1, 1, North });
    std::cout << (e41.first ? "true" : "false") << " " << (int)e41.second << std::endl;
    std::cout << (e42.first ? "true" : "false") << " " << (int)e42.second << std::endl;
    std::cout << (e43.first ? "true" : "false") << " " << (int)e43.second << std::endl;
    std::cout << std::endl;
    std::cout << mg4.size << std::endl;
    std::cout << std::endl;

    std::cout << std::endl;
    std::cout << "=======Testing D* Lite for 4-way Graph=======" << std::endl;
    std::cout << std::endl;

    solver1.initialize();
    decltype(mg1)::NodeId id_goal = mg1.getGoalNodeId();
    std::cout << "The start is " << mg1.getStartNodeId() << std::endl;
    std::cout << "The goal is " << id_goal << std::endl;
    std::cout << std::endl;

    while (solver1.getCurrentNodeId() != id_goal) {
        std::cout << mg1.coordByNodeId(solver1.getCurrentNodeId()) << std::endl;
        solver1.preSense();
        solver1.postSense(std::vector<Coordinates>());
    }
    std::cout << id_goal << std::endl;

    std::cout << std::endl;
    std::cout << "=======Testing D* Lite for 6-way Graph=======" << std::endl;
    std::cout << std::endl;

    solver4.initialize();
    decltype(mg4)::NodeId id_goal4 = mg4.getGoalNodeId();
    std::cout << "The start is " << mg4.getStartNodeId() << std::endl;
    std::cout << "The goal is " << id_goal4 << std::endl;
    std::cout << std::endl;

    while (solver4.getCurrentNodeId() != id_goal4) {
        std::cout << mg4.coordByNodeId(solver4.getCurrentNodeId()) << std::endl;
        solver4.preSense();
        solver4.postSense(std::vector<Coordinates>());
    }
    std::cout << id_goal4 << std::endl;

    std::cout << std::endl;
    std::cout << "=======Testing Dynamic Search Using D* Lite for 4-way Graph=======" << std::endl;
    std::cout << std::endl;

    Maze maze(max_maze_width, max_maze_width);
    Coordinates goal_coordinates = reference_maze.getGoal();
    Utility::loadEmptyMaze(max_maze_width, max_maze_width, goal_coordinates.x, goal_coordinates.y, maze);
    maze.setCheckedWall({ 0, 0, North });
    maze.setCheckedWall({ 0, 0, East });
    maze.setCheckedWall({ 0, 0, South });
    maze.setCheckedWall({ 0, 0, West });
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
        sense(maze, reference_maze, mg5.coordByNodeId(id), changed_coordinates);
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
    return 0;
}
