#pragma once

#include <stdint.h>

// Must stay free of Arduino.h so it can be tested on a laptop.

enum Dir : uint8_t { NORTH = 0, EAST = 1, SOUTH = 2, WEST = 3 };

class Maze {
public:
    static constexpr uint8_t MAX_SIZE = 9;
    static constexpr uint8_t UNREACHABLE = 255;

    explicit Maze(uint8_t size);

    void clear();

    // True when (x, y) is a real cell of this maze.
    bool inBounds(uint8_t x, uint8_t y) const;

    // Walls are shared: setting the north wall of (x, y) also sets the south
    // wall of the cell above it. Out-of-range cells are ignored.
    void setWall(uint8_t x, uint8_t y, Dir d);

    // Removes a wall again, on both sides. The outer boundary of the maze is
    // not stored as bits, so it can never be opened by this call.
    void clearWall(uint8_t x, uint8_t y, Dir d);

    // Out-of-range cells report "wall", so callers can never walk off the map.
    bool hasWall(uint8_t x, uint8_t y, Dir d) const;

    // The cell one step in direction d. Returns false if that leaves the maze.
    bool neighbour(uint8_t x, uint8_t y, Dir d, uint8_t &nx, uint8_t &ny) const;

    // BFS outward from the goal. An out-of-range goal leaves every cell
    // UNREACHABLE instead of writing outside the arrays.
    void floodFill(uint8_t goalX, uint8_t goalY);
    uint8_t distance(uint8_t x, uint8_t y) const;

    // Legacy chooser. Kept so existing callers and tests keep working, but it
    // cannot report failure: when the goal is unreachable it returns NORTH
    // even if a wall is there. Prefer the four-argument version below.
    Dir bestDirection(uint8_t x, uint8_t y) const;

    // Picks the neighbouring cell that is strictly closer to the goal, breaking
    // ties in favour of driving straight on (fewer turns = faster run).
    // Returns false - and leaves 'out' alone - when there is no such cell,
    // i.e. the goal is unreachable, the robot is already on it, or (x, y) is
    // out of range. Call floodFill() first.
    bool bestDirection(uint8_t x, uint8_t y, Dir heading, Dir &out) const;

    uint8_t size() const { return size_; }

private:
    uint8_t size_;
    uint8_t walls_[MAX_SIZE][MAX_SIZE]; // bit per direction
    uint8_t dist_[MAX_SIZE][MAX_SIZE];
};
