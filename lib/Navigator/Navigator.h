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
        // Neither of these moved the robot, and they ask for different things:
        // Turned means TURN THE ROBOT, NeedsSensing means READ THE SENSORS.
        // New results go at the end so the ones above keep their values.
        Turned,       // now facing 'chosen'; re-sense, then call step() again
        NeedsSensing, // no fresh reading for this pose; recordWalls() first
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
    // 'chosen' is written for Moved and Turned only.
    //
    // The sensors miss the cell behind the robot, which an empty map reads as
    // open, so this never moves that way on trust: it returns Turned to have
    // the robot face it and be re-sensed first. Once recordWalls() has been
    // called at all, it also refuses to move on readings belonging to an older
    // pose, returning NeedsSensing. Callers driving from a fully known map
    // never sense, and are unaffected.
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

    // Where the robot was pointing when recordWalls() last ran for the cell it
    // is in: the one direction those readings never cover is
    // behind(sensedHeading_).
    bool sensedValid_;
    Dir sensedHeading_;

    // Set by the first recordWalls() and never cleared. It marks a caller that
    // drives from its sensors, which must confirm a direction before moving
    // there; it stays false for known-map callers, which changes nothing.
    bool everSensed_;
};
