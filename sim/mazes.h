#pragma once

#include <stdint.h>

#include <string>
#include <vector>

#include "Maze.h"

// A maze is drawn as plain text so you can edit it by hand. Each cell is five
// characters wide. A '-' in a horizontal border or a '|' in a cell row means
// "wall here"; a space means "open passage".
//
//   +-----+-----+      x grows to the EAST (right)
//   |     |     |      y grows to the NORTH (up)
//   +-----+     +      so the TOP line of the drawing is the highest y
//   |           |
//   +-----+-----+
struct MazeSpec {
    const char *name;
    const char *description;
    const char *art;
    uint8_t startX, startY;
    Dir startHeading;
    uint8_t goalX, goalY;
};

const std::vector<MazeSpec> &builtInMazes();
const MazeSpec *findMaze(const std::string &name);

// Works out the maze size from a drawing. Returns 0 and sets 'error' if the
// drawing is malformed.
uint8_t mazeArtSize(const std::string &art, std::string &error);

// Fills 'out' (already constructed at the right size) with the walls in the
// drawing. Returns false and sets 'error' on a malformed drawing.
bool parseMazeArt(const std::string &art, Maze &out, std::string &error);
