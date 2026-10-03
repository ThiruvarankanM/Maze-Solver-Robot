#include "Maze.h"

static const int8_t DX[4] = {0, 1, 0, -1};
static const int8_t DY[4] = {1, 0, -1, 0};

static Dir opposite(Dir d) { return static_cast<Dir>((d + 2) % 4); }

Maze::Maze(uint8_t size) : size_(size > MAX_SIZE ? MAX_SIZE : size) { clear(); }

void Maze::clear()
{
    for (uint8_t x = 0; x < MAX_SIZE; x++) {
        for (uint8_t y = 0; y < MAX_SIZE; y++) {
            walls_[x][y] = 0;
            dist_[x][y] = UNREACHABLE;
        }
    }
}

bool Maze::neighbour(uint8_t x, uint8_t y, Dir d, uint8_t &nx, uint8_t &ny) const
{
    int8_t tx = x + DX[d];
    int8_t ty = y + DY[d];
    if (tx < 0 || ty < 0 || tx >= size_ || ty >= size_) {
        return false;
    }
    nx = tx;
    ny = ty;
    return true;
}

void Maze::setWall(uint8_t x, uint8_t y, Dir d)
{
    walls_[x][y] |= 1 << d;
    uint8_t nx, ny;
    if (neighbour(x, y, d, nx, ny)) {
        walls_[nx][ny] |= 1 << opposite(d);
    }
}

bool Maze::hasWall(uint8_t x, uint8_t y, Dir d) const
{
    uint8_t nx, ny;
    return (walls_[x][y] & (1 << d)) || !neighbour(x, y, d, nx, ny);
}

void Maze::floodFill(uint8_t goalX, uint8_t goalY)
{
    for (uint8_t x = 0; x < size_; x++) {
        for (uint8_t y = 0; y < size_; y++) {
            dist_[x][y] = UNREACHABLE;
        }
    }

    // BFS outward from the goal; each cell gets its step count
    uint8_t queue[MAX_SIZE * MAX_SIZE];
    uint8_t head = 0, tail = 0;
    dist_[goalX][goalY] = 0;
    queue[tail++] = goalX * MAX_SIZE + goalY;

    while (head < tail) {
        uint8_t x = queue[head] / MAX_SIZE;
        uint8_t y = queue[head] % MAX_SIZE;
        head++;
        for (uint8_t d = 0; d < 4; d++) {
            uint8_t nx, ny;
            if (hasWall(x, y, static_cast<Dir>(d)) || !neighbour(x, y, static_cast<Dir>(d), nx, ny)) {
                continue;
            }
            if (dist_[nx][ny] == UNREACHABLE) {
                dist_[nx][ny] = dist_[x][y] + 1;
                queue[tail++] = nx * MAX_SIZE + ny;
            }
        }
    }
}

uint8_t Maze::distance(uint8_t x, uint8_t y) const { return dist_[x][y]; }

Dir Maze::bestDirection(uint8_t x, uint8_t y) const
{
    Dir best = NORTH;
    uint8_t bestDist = UNREACHABLE;
    for (uint8_t d = 0; d < 4; d++) {
        uint8_t nx, ny;
        if (hasWall(x, y, static_cast<Dir>(d)) || !neighbour(x, y, static_cast<Dir>(d), nx, ny)) {
            continue;
        }
        if (dist_[nx][ny] < bestDist) {
            bestDist = dist_[nx][ny];
            best = static_cast<Dir>(d);
        }
    }
    return best;
}
