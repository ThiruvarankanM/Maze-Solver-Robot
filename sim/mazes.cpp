#include "mazes.h"

#include <cstdio>

static const char *ART_OPEN =
    "+-----+-----+-----+-----+\n"
    "|                       |\n"
    "+     +     +     +     +\n"
    "|                       |\n"
    "+     +     +     +     +\n"
    "|                       |\n"
    "+     +     +     +     +\n"
    "|                       |\n"
    "+-----+-----+-----+-----+\n";

static const char *ART_OBSTACLES =
    "+-----+-----+-----+-----+\n"
    "|           |           |\n"
    "+     +     +     +     +\n"
    "|           |           |\n"
    "+     +     +     +     +\n"
    "|           |           |\n"
    "+     +     +     +     +\n"
    "|                       |\n"
    "+-----+-----+-----+-----+\n";

static const char *ART_DEADEND =
    "+-----+-----+-----+-----+\n"
    "|                       |\n"
    "+     +     +     +     +\n"
    "|                       |\n"
    "+     +     +     +     +\n"
    "|                       |\n"
    "+     +-----+-----+-----+\n"
    "|                       |\n"
    "+-----+-----+-----+-----+\n";

static const char *ART_MULTI =
    "+-----+-----+-----+-----+\n"
    "|                       |\n"
    "+     +-----+-----+     +\n"
    "|     |           |     |\n"
    "+     +     +     +     +\n"
    "|     |           |     |\n"
    "+     +-----+-----+     +\n"
    "|                       |\n"
    "+-----+-----+-----+-----+\n";

static const char *ART_UNREACHABLE =
    "+-----+-----+-----+-----+\n"
    "|                 |     |\n"
    "+     +     +     +-----+\n"
    "|                       |\n"
    "+     +     +     +     +\n"
    "|                       |\n"
    "+     +     +     +     +\n"
    "|                       |\n"
    "+-----+-----+-----+-----+\n";

const std::vector<MazeSpec> &builtInMazes()
{
    static const std::vector<MazeSpec> mazes = {
        {"open", "4x4 with no interior walls - shortest path is a staircase",
         ART_OPEN, 0, 0, NORTH, 3, 3},
        {"obstacles", "A barrier forces a long detour along the bottom row",
         ART_OBSTACLES, 0, 3, SOUTH, 3, 3},
        {"deadend", "Robot starts inside a dead-end corridor and must back out",
         ART_DEADEND, 3, 0, WEST, 0, 3},
        {"multi", "A sealed island in the middle - two routes of equal length",
         ART_MULTI, 0, 0, NORTH, 3, 3},
        {"unreachable", "The goal cell is walled off - must report failure",
         ART_UNREACHABLE, 0, 0, NORTH, 3, 3},
    };
    return mazes;
}

const MazeSpec *findMaze(const std::string &name)
{
    for (const MazeSpec &m : builtInMazes()) {
        if (name == m.name) {
            return &m;
        }
    }
    return nullptr;
}

static std::vector<std::string> splitLines(const std::string &art)
{
    std::vector<std::string> lines;
    std::string current;
    for (char c : art) {
        if (c == '\n') {
            lines.push_back(current);
            current.clear();
        } else if (c != '\r') {
            current += c;
        }
    }
    if (!current.empty()) {
        lines.push_back(current);
    }
    while (!lines.empty() && lines.back().empty()) {
        lines.pop_back();
    }
    return lines;
}

static bool checkArt(const std::string &art, std::vector<std::string> &lines,
                     uint8_t &size, std::string &error)
{
    lines = splitLines(art);
    char msg[160];

    if (lines.size() < 3 || lines.size() % 2 == 0) {
        snprintf(msg, sizeof(msg),
                 "a maze drawing needs an odd number of lines (3, 5, 7, ...), got %zu",
                 lines.size());
        error = msg;
        return false;
    }

    const size_t n = (lines.size() - 1) / 2;
    if (n > Maze::MAX_SIZE) {
        snprintf(msg, sizeof(msg), "maze is %zu cells wide, the maximum is %u", n,
                 static_cast<unsigned>(Maze::MAX_SIZE));
        error = msg;
        return false;
    }

    const size_t expected = 6 * n + 1;
    for (size_t i = 0; i < lines.size(); i++) {
        if (lines[i].size() != expected) {
            snprintf(msg, sizeof(msg),
                     "line %zu is %zu characters, expected %zu (each cell is 5 wide)",
                     i + 1, lines[i].size(), expected);
            error = msg;
            return false;
        }
    }

    size = static_cast<uint8_t>(n);
    return true;
}

uint8_t mazeArtSize(const std::string &art, std::string &error)
{
    std::vector<std::string> lines;
    uint8_t size = 0;
    if (!checkArt(art, lines, size, error)) {
        return 0;
    }
    return size;
}

bool parseMazeArt(const std::string &art, Maze &out, std::string &error)
{
    std::vector<std::string> lines;
    uint8_t size = 0;
    if (!checkArt(art, lines, size, error)) {
        return false;
    }
    if (size != out.size()) {
        char msg[120];
        snprintf(msg, sizeof(msg), "drawing is %u cells wide but the Maze is %u", size,
                 out.size());
        error = msg;
        return false;
    }

    out.clear();
    for (uint8_t y = 0; y < size; y++) {
        const size_t r = static_cast<size_t>(size - 1 - y); // top line is the highest y
        for (uint8_t x = 0; x < size; x++) {
            const size_t col = 6u * x;
            if (lines[2 * r][col + 3] == '-') {
                out.setWall(x, y, NORTH);
            }
            if (lines[2 * r + 2][col + 3] == '-') {
                out.setWall(x, y, SOUTH);
            }
            if (lines[2 * r + 1][col] == '|') {
                out.setWall(x, y, WEST);
            }
            if (lines[2 * r + 1][col + 6] == '|') {
                out.setWall(x, y, EAST);
            }
        }
    }
    return true;
}
