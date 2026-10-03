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
    void setWall(uint8_t x, uint8_t y, Dir d);
    bool hasWall(uint8_t x, uint8_t y, Dir d) const;

    void floodFill(uint8_t goalX, uint8_t goalY);
    uint8_t distance(uint8_t x, uint8_t y) const;
    Dir bestDirection(uint8_t x, uint8_t y) const;

    uint8_t size() const { return size_; }

private:
    bool neighbour(uint8_t x, uint8_t y, Dir d, uint8_t &nx, uint8_t &ny) const;

    uint8_t size_;
    uint8_t walls_[MAX_SIZE][MAX_SIZE]; // bit per direction
    uint8_t dist_[MAX_SIZE][MAX_SIZE];
};
