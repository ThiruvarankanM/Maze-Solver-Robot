#pragma once

#include <string>

#include "Maze.h"

const char *dirName(Dir d);
char dirArrow(Dir d);

// Draws the maze with the flood fill distance inside every cell. Pass -1 for
// robotX / goalX / startX to leave that marker out.
std::string renderMaze(const Maze &m, int robotX, int robotY, Dir heading, int goalX,
                       int goalY, int startX, int startY);
