#include "Navigator.h"

Navigator::Navigator(Maze &map, uint8_t x, uint8_t y, Dir heading)
    : map_(map), x_(x), y_(y), heading_(heading), moves_(0)
{
}

void Navigator::setPose(uint8_t x, uint8_t y, Dir heading)
{
    x_ = x;
    y_ = y;
    heading_ = heading;
    moves_ = 0;
}

void Navigator::recordWalls(bool left, bool front, bool right)
{
    const Dir sides[3] = {leftOf(heading_), heading_, rightOf(heading_)};
    const bool seen[3] = {left, front, right};

    for (uint8_t i = 0; i < 3; i++) {
        if (seen[i]) {
            map_.setWall(x_, y_, sides[i]);
        } else {
            map_.clearWall(x_, y_, sides[i]);
        }
    }
}

Navigator::Result Navigator::step(uint8_t goalX, uint8_t goalY, Dir &chosen)
{
    if (!map_.inBounds(goalX, goalY) || !map_.inBounds(x_, y_)) {
        return Result::InvalidGoal;
    }
    if (x_ == goalX && y_ == goalY) {
        return Result::Arrived;
    }

    map_.floodFill(goalX, goalY);

    if (map_.distance(x_, y_) == Maze::UNREACHABLE) {
        return Result::Unreachable;
    }

    Dir d;
    if (!map_.bestDirection(x_, y_, heading_, d)) {
        return Result::Blocked;
    }

    uint8_t nx, ny;
    if (!map_.neighbour(x_, y_, d, nx, ny)) {
        return Result::Blocked; // belt and braces: never step off the map
    }

    heading_ = d;
    x_ = nx;
    y_ = ny;
    moves_++;
    chosen = d;
    return Result::Moved;
}
