#include "Navigator.h"

Navigator::Navigator(Maze &map, uint8_t x, uint8_t y, Dir heading)
    : map_(map), x_(x), y_(y), heading_(heading), moves_(0), sensedValid_(false),
      sensedHeading_(heading), everSensed_(false)
{
}

void Navigator::setPose(uint8_t x, uint8_t y, Dir heading)
{
    x_ = x;
    y_ = y;
    heading_ = heading;
    moves_ = 0;
    sensedValid_ = false; // nothing has been sensed from the new pose yet
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

    // Remember the orientation these three readings were taken at, so step()
    // knows which single direction is still unsensed.
    sensedValid_ = true;
    sensedHeading_ = heading_;
    everSensed_ = true; // from now on this caller must sense before it moves
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

    // The sensors never see behind the robot, so that wall is unknown and an
    // empty map reads it as open. Never move there on trust: turn to face it
    // and let the caller take a fresh front reading first. On the real robot
    // this is also the only physically possible way to get there, as it cannot
    // drive backwards.
    if (everSensed_) {
        // This caller drives from its sensors, so it must never move on map
        // data it has not just confirmed. After a turn - or after a move into
        // a new cell - the three readings belong to an older pose, so ask for
        // a fresh observation instead of trusting them.
        if (!sensedValid_) {
            return Result::NeedsSensing;
        }
        if (d == behind(sensedHeading_)) {
            heading_ = d;
            sensedValid_ = false; // left/front/right now point somewhere else
            chosen = d;
            return Result::Turned;
        }
    }

    heading_ = d;
    x_ = nx;
    y_ = ny;
    moves_++;
    sensedValid_ = false; // the new cell has not been sensed yet
    chosen = d;
    return Result::Moved;
}
