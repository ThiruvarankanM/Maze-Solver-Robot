#include "render.h"

#include <cstdio>

const char *dirName(Dir d)
{
    switch (d) {
    case NORTH: return "NORTH";
    case EAST:  return "EAST";
    case SOUTH: return "SOUTH";
    case WEST:  return "WEST";
    }
    return "?";
}

char dirArrow(Dir d)
{
    switch (d) {
    case NORTH: return '^';
    case EAST:  return '>';
    case SOUTH: return 'v';
    case WEST:  return '<';
    }
    return '?';
}

// Five characters: one marker column, then the distance right-aligned in
// three, then a space. e.g. "^  6 " or "G  0 " or "   12".
static std::string cell(const Maze &m, uint8_t x, uint8_t y, int robotX, int robotY,
                        Dir heading, int goalX, int goalY, int startX, int startY)
{
    char marker = ' ';
    if (robotX == static_cast<int>(x) && robotY == static_cast<int>(y)) {
        marker = dirArrow(heading);
    } else if (goalX == static_cast<int>(x) && goalY == static_cast<int>(y)) {
        marker = 'G';
    } else if (startX == static_cast<int>(x) && startY == static_cast<int>(y)) {
        marker = 'S';
    }

    const uint8_t d = m.distance(x, y);
    char buf[8];
    if (d == Maze::UNREACHABLE) {
        snprintf(buf, sizeof(buf), "%c  - ", marker);
    } else {
        snprintf(buf, sizeof(buf), "%c%3u ", marker, static_cast<unsigned>(d));
    }
    return std::string(buf);
}

std::string renderMaze(const Maze &m, int robotX, int robotY, Dir heading, int goalX,
                       int goalY, int startX, int startY)
{
    const uint8_t n = m.size();
    std::string out;

    for (int y = static_cast<int>(n) - 1; y >= 0; y--) {
        const uint8_t uy = static_cast<uint8_t>(y);

        out += '+';
        for (uint8_t x = 0; x < n; x++) {
            out += m.hasWall(x, uy, NORTH) ? "-----" : "     ";
            out += '+';
        }
        out += '\n';

        for (uint8_t x = 0; x < n; x++) {
            out += m.hasWall(x, uy, WEST) ? '|' : ' ';
            out += cell(m, x, uy, robotX, robotY, heading, goalX, goalY, startX, startY);
        }
        out += '|';
        out += '\n';
    }

    out += '+';
    for (uint8_t x = 0; x < n; x++) {
        out += m.hasWall(x, 0, SOUTH) ? "-----" : "     ";
        out += '+';
    }
    out += '\n';

    return out;
}
