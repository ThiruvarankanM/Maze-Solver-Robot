#pragma once

#include <stdint.h>

#include "Maze.h"

// Tracks where the robot thinks it is and decides what to do next.
//
// Must stay free of Arduino.h: the same class runs in the Mac simulator and,
// later, on the Mega. It never touches motors or sensors itself. It only
// reads a map, updates a pose, and reports a decision; something else (the
// simulator, or a Drive class) actually performs the movement.
class Navigator {
public:
    enum class Result : uint8_t {
        Arrived,     // standing on the goal, nothing to do
        Moved,       // moved one cell; 'chosen' holds the direction taken
        Unreachable, // no route to the goal exists on the current map
        Blocked,     // no legal move gets closer (safety net, should not happen)
        InvalidGoal, // the goal, or the robot's own pose, is outside the maze
    };

    // 'map' is the robot's BELIEF about the walls, not the real maze. The
    // Navigator keeps a reference to it, so it must outlive the Navigator.
    Navigator(Maze &map, uint8_t x, uint8_t y, Dir heading);

    uint8_t x() const { return x_; }
    uint8_t y() const { return y_; }
    Dir heading() const { return heading_; }
    uint16_t moveCount() const { return moves_; }

    void setPose(uint8_t x, uint8_t y, Dir heading);

    // What the three wall sensors see, in the ROBOT's frame of reference.
    // Writes the result into the belief map for the current cell. Passing
    // false for a side also erases a wall recorded there earlier, so one bad
    // reading cannot block a corridor for the rest of the run.
    void recordWalls(bool left, bool front, bool right);

    // Runs flood fill towards the goal, picks a direction and moves one cell.
    // 'chosen' is only written when the result is Moved.
    Result step(uint8_t goalX, uint8_t goalY, Dir &chosen);

    static Dir leftOf(Dir d) { return static_cast<Dir>((d + 3) % 4); }
    static Dir rightOf(Dir d) { return static_cast<Dir>((d + 1) % 4); }
    static Dir behind(Dir d) { return static_cast<Dir>((d + 2) % 4); }

private:
    Maze &map_;
    uint8_t x_;
    uint8_t y_;
    Dir heading_;
    uint16_t moves_;
};
