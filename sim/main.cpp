// Maze simulator - runs on a laptop, no Arduino required.
//
// It links the REAL lib/Maze and lib/Navigator code that runs on the Mega.
// Only the walls, the sensors and the movement are faked here.
//
// IMPORTANT: this proves the navigation logic. It proves nothing at all about
// motors, encoders, gyro drift or ultrasonic sensors.

#include <sys/select.h>
#include <unistd.h>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include "Maze.h"
#include "Navigator.h"
#include "mazes.h"
#include "render.h"

struct Options {
    std::string mazeName = "obstacles";
    std::string file;
    bool explore = false;
    bool autoRun = false;
    unsigned delayMs = 350;
    int maxSteps = -1; // -1 = work it out from the maze size
    bool haveStart = false, haveGoal = false, haveHeading = false;
    uint8_t startX = 0, startY = 0, goalX = 0, goalY = 0;
    Dir heading = NORTH;
};

static void usage()
{
    printf(
        "Maze simulator\n"
        "\n"
        "Usage: program [options]\n"
        "\n"
        "  --list              show the built-in mazes and exit\n"
        "  --maze NAME         pick a built-in maze (default: obstacles)\n"
        "  --file PATH         load a maze drawing from a text file instead\n"
        "  --start X,Y         starting cell        (default: the maze's own)\n"
        "  --heading N|E|S|W   starting direction   (default: the maze's own)\n"
        "  --goal X,Y          target cell          (default: the maze's own)\n"
        "  --explore           robot begins with an empty map and feels its way\n"
        "                      (default: it is given the full wall layout)\n"
        "  --auto              run straight through without stopping\n"
        "  --delay MS          frame delay in auto mode (default 350)\n"
        "  --max-steps N       give up after N moves\n"
        "  --help              this text\n"
        "\n"
        "Keys while it is paused:\n"
        "  Enter  one step      r  run    x  reset    q  quit\n"
        "Press Enter while it is running to pause again.\n");
}

static bool parseXY(const char *s, uint8_t &x, uint8_t &y)
{
    int a = 0, b = 0;
    if (sscanf(s, "%d,%d", &a, &b) != 2) {
        return false;
    }
    if (a < 0 || b < 0 || a > 255 || b > 255) {
        return false;
    }
    x = static_cast<uint8_t>(a);
    y = static_cast<uint8_t>(b);
    return true;
}

static bool parseHeading(const char *s, Dir &d)
{
    if (!s[0] || s[1]) {
        return false;
    }
    switch (s[0]) {
    case 'N': case 'n': d = NORTH; return true;
    case 'E': case 'e': d = EAST;  return true;
    case 'S': case 's': d = SOUTH; return true;
    case 'W': case 'w': d = WEST;  return true;
    default: return false;
    }
}

static bool readFile(const std::string &path, std::string &out)
{
    FILE *f = fopen(path.c_str(), "rb");
    if (!f) {
        return false;
    }
    char buf[4096];
    size_t n;
    while ((n = fread(buf, 1, sizeof(buf), f)) > 0) {
        out.append(buf, n);
    }
    fclose(f);
    return true;
}

// True when the program is being driven by a human at a terminal, rather than
// by a pipe or a script. Without this, end-of-file on a pipe looks exactly like
// a keypress and the run stops on the first frame.
static bool interactive() { return isatty(STDIN_FILENO) != 0; }

// True when a line is waiting on stdin, without blocking if there is not.
static bool inputWaiting()
{
    if (!interactive()) {
        return false;
    }
    fd_set set;
    FD_ZERO(&set);
    FD_SET(STDIN_FILENO, &set);
    struct timeval tv = {0, 0};
    return select(STDIN_FILENO + 1, &set, nullptr, nullptr, &tv) > 0;
}

static std::string readLine()
{
    char buf[64];
    if (!fgets(buf, sizeof(buf), stdin)) {
        return "q";
    }
    std::string s(buf);
    while (!s.empty() && (s.back() == '\n' || s.back() == '\r')) {
        s.pop_back();
    }
    return s;
}

static void copyWalls(const Maze &from, Maze &to)
{
    to.clear();
    for (uint8_t x = 0; x < from.size(); x++) {
        for (uint8_t y = 0; y < from.size(); y++) {
            for (uint8_t d = 0; d < 4; d++) {
                if (from.hasWall(x, y, static_cast<Dir>(d))) {
                    to.setWall(x, y, static_cast<Dir>(d));
                }
            }
        }
    }
}

// What the three wall sensors would report, read off the real maze.
static void sense(const Maze &truth, const Navigator &nav, bool &left, bool &front,
                  bool &right)
{
    const Dir h = nav.heading();
    left = truth.hasWall(nav.x(), nav.y(), Navigator::leftOf(h));
    front = truth.hasWall(nav.x(), nav.y(), h);
    right = truth.hasWall(nav.x(), nav.y(), Navigator::rightOf(h));
}

int main(int argc, char **argv)
{
    Options opt;

    for (int i = 1; i < argc; i++) {
        const char *a = argv[i];
        const bool hasNext = (i + 1 < argc);
        if (!strcmp(a, "--help") || !strcmp(a, "-h")) {
            usage();
            return 0;
        } else if (!strcmp(a, "--list")) {
            printf("Built-in mazes:\n");
            for (const MazeSpec &m : builtInMazes()) {
                printf("  %-12s %s\n", m.name, m.description);
                printf("  %-12s start (%u,%u) facing %s, goal (%u,%u)\n", "", m.startX,
                       m.startY, dirName(m.startHeading), m.goalX, m.goalY);
            }
            return 0;
        } else if (!strcmp(a, "--maze") && hasNext) {
            opt.mazeName = argv[++i];
        } else if (!strcmp(a, "--file") && hasNext) {
            opt.file = argv[++i];
        } else if (!strcmp(a, "--start") && hasNext) {
            if (!parseXY(argv[++i], opt.startX, opt.startY)) {
                fprintf(stderr, "bad --start, expected X,Y\n");
                return 2;
            }
            opt.haveStart = true;
        } else if (!strcmp(a, "--goal") && hasNext) {
            if (!parseXY(argv[++i], opt.goalX, opt.goalY)) {
                fprintf(stderr, "bad --goal, expected X,Y\n");
                return 2;
            }
            opt.haveGoal = true;
        } else if (!strcmp(a, "--heading") && hasNext) {
            if (!parseHeading(argv[++i], opt.heading)) {
                fprintf(stderr, "bad --heading, expected N, E, S or W\n");
                return 2;
            }
            opt.haveHeading = true;
        } else if (!strcmp(a, "--explore")) {
            opt.explore = true;
        } else if (!strcmp(a, "--auto")) {
            opt.autoRun = true;
        } else if (!strcmp(a, "--delay") && hasNext) {
            opt.delayMs = static_cast<unsigned>(atoi(argv[++i]));
        } else if (!strcmp(a, "--max-steps") && hasNext) {
            opt.maxSteps = atoi(argv[++i]);
        } else {
            fprintf(stderr, "unknown option: %s   (try --help)\n", a);
            return 2;
        }
    }

    // ---- load the maze drawing -------------------------------------------
    std::string art;
    uint8_t startX, startY, goalX, goalY;
    Dir startHeading;

    if (!opt.file.empty()) {
        if (!readFile(opt.file, art)) {
            fprintf(stderr, "cannot read %s\n", opt.file.c_str());
            return 2;
        }
        startX = startY = 0;
        startHeading = NORTH;
        goalX = goalY = 0; // sensible defaults, almost always overridden
    } else {
        const MazeSpec *spec = findMaze(opt.mazeName);
        if (!spec) {
            fprintf(stderr, "no built-in maze called '%s'. Try --list\n",
                    opt.mazeName.c_str());
            return 2;
        }
        art = spec->art;
        startX = spec->startX;
        startY = spec->startY;
        startHeading = spec->startHeading;
        goalX = spec->goalX;
        goalY = spec->goalY;
    }

    std::string err;
    const uint8_t size = mazeArtSize(art, err);
    if (size == 0) {
        fprintf(stderr, "bad maze drawing: %s\n", err.c_str());
        return 2;
    }

    Maze truth(size);
    if (!parseMazeArt(art, truth, err)) {
        fprintf(stderr, "bad maze drawing: %s\n", err.c_str());
        return 2;
    }

    if (opt.haveStart) { startX = opt.startX; startY = opt.startY; }
    if (opt.haveGoal)  { goalX = opt.goalX;   goalY = opt.goalY; }
    if (opt.haveHeading) { startHeading = opt.heading; }

    // ---- validate the coordinates the user asked for ----------------------
    if (!truth.inBounds(startX, startY)) {
        fprintf(stderr, "FAILED: start (%u,%u) is outside a %ux%u maze\n", startX,
                startY, size, size);
        return 1;
    }
    if (!truth.inBounds(goalX, goalY)) {
        fprintf(stderr, "FAILED: goal (%u,%u) is outside a %ux%u maze\n", goalX, goalY,
                size, size);
        return 1;
    }

    const int maxSteps = (opt.maxSteps > 0) ? opt.maxSteps : (size * size * 8);

    // ---- set the robot up -------------------------------------------------
    Maze belief(size); // what the robot THINKS the walls are
    Navigator nav(belief, startX, startY, startHeading);
    std::vector<std::pair<uint8_t, uint8_t>> path;

    auto reset = [&]() {
        if (opt.explore) {
            belief.clear(); // knows nothing except the outer boundary
        } else {
            copyWalls(truth, belief);
        }
        nav.setPose(startX, startY, startHeading);
        path.clear();
        path.push_back({startX, startY});
    };
    reset();

    printf("Maze:    %s (%ux%u)\n", opt.file.empty() ? opt.mazeName.c_str() : opt.file.c_str(),
           size, size);
    printf("Mode:    %s\n",
           opt.explore ? "explore (map starts empty, walls sensed on the way)"
                       : "known map (full wall layout given to the robot)");
    printf("Start:   (%u,%u) facing %s\n", startX, startY, dirName(startHeading));
    printf("Goal:    (%u,%u)\n", goalX, goalY);
    if (!opt.autoRun) {
        printf("\nKeys: Enter = one step, r = run, x = reset, q = quit\n");
    }

    bool running = opt.autoRun;
    int steps = 0;
    const char *verdict = nullptr;
    char illegalMove[128]; // holds the verdict text if a move crosses a real wall
    std::string lastDecision = "(not moved yet)";

    while (true) {
        if (opt.explore) {
            bool l, f, r;
            sense(truth, nav, l, f, r);
            nav.recordWalls(l, f, r);
        }
        belief.floodFill(goalX, goalY);

        if (running && interactive()) {
            printf("\033[2J\033[H");
        }
        printf("\n%s", renderMaze(belief, nav.x(), nav.y(), nav.heading(), goalX, goalY,
                                  startX, startY)
                           .c_str());

        const uint8_t here = belief.distance(nav.x(), nav.y());
        printf("step %-3d  at (%u,%u) facing %-5s  distance to goal: ", steps, nav.x(),
               nav.y(), dirName(nav.heading()));
        if (here == Maze::UNREACHABLE) {
            printf("unreachable\n");
        } else {
            printf("%u\n", here);
        }
        printf("last decision: %s\n", lastDecision.c_str());

        if (nav.x() == goalX && nav.y() == goalY) {
            verdict = "SUCCESS";
            break;
        }

        if (running && inputWaiting()) {
            readLine();
            running = false;
            printf("\n-- paused --\n");
        }

        if (!running) {
            printf("> ");
            fflush(stdout);
            const std::string cmd = readLine();
            if (cmd == "q") {
                verdict = "QUIT";
                break;
            }
            if (cmd == "x") {
                reset();
                steps = 0;
                lastDecision = "(reset)";
                continue;
            }
            if (cmd == "r") {
                running = true;
            } else if (!cmd.empty()) {
                printf("unknown key '%s'. Enter = step, r = run, x = reset, q = quit\n",
                       cmd.c_str());
                continue;
            }
        }

        if (steps >= maxSteps) {
            verdict = "FAILED: gave up after the step limit - the robot is going in circles";
            break;
        }

        const uint8_t prevX = nav.x();
        const uint8_t prevY = nav.y();
        Dir chosen = NORTH;
        const Navigator::Result res = nav.step(goalX, goalY, chosen);
        steps++;

        // The robot decides using its BELIEF about the walls. Check the result
        // against the REAL maze: without this, a move straight through a solid
        // wall is indistinguishable from a legal one and still counts towards
        // SUCCESS. This is the same check the unit tests make on every move.
        if (res == Navigator::Result::Moved &&
            (truth.hasWall(prevX, prevY, chosen) || !truth.inBounds(nav.x(), nav.y()))) {
            snprintf(illegalMove, sizeof(illegalMove),
                     "FAILED: illegal move - went %s from (%u,%u) through a real wall",
                     dirName(chosen), prevX, prevY);
            verdict = illegalMove;
            break;
        }

        switch (res) {
        case Navigator::Result::Moved:
            lastDecision = std::string("move ") + dirName(chosen);
            path.push_back({nav.x(), nav.y()});
            break;
        case Navigator::Result::NeedsSensing:
            // Cannot happen while the loop above senses on every pass, so if it
            // ever does the loop itself is wrong. Stop loudly rather than spin
            // or let the run end up counted as a success.
            verdict = "FAILED: the navigator wants a fresh sensor reading before it "
                      "moves - recordWalls() was not called for the current pose";
            break;
        case Navigator::Result::Turned:
            // Turned on the spot, nothing moved: not a move, so it is kept out
            // of the path and out of moveCount(). The top of the loop re-reads
            // the wall sensors, which is the whole point - the front sensor now
            // looks at the cell the robot wants to enter.
            lastDecision = std::string("turn to face ") + dirName(chosen) +
                           " (re-checking the front sensor before moving)";
            break;
        case Navigator::Result::Arrived:
            verdict = "SUCCESS";
            break;
        case Navigator::Result::Unreachable:
            verdict = "FAILED: the goal cannot be reached from here - no route exists";
            break;
        case Navigator::Result::Blocked:
            verdict = "FAILED: no legal move gets the robot any closer";
            break;
        case Navigator::Result::InvalidGoal:
            verdict = "FAILED: the goal or the robot's position is outside the maze";
            break;
        }
        if (verdict) {
            break;
        }

        if (running && opt.delayMs > 0) {
            usleep(opt.delayMs * 1000);
        }
    }

    printf("\n==================================================\n");
    printf("%s\n", verdict ? verdict : "STOPPED");
    printf("moves taken: %u\n", nav.moveCount());
    printf("path: ");
    for (size_t i = 0; i < path.size(); i++) {
        printf("%s(%u,%u)", i ? " -> " : "", path[i].first, path[i].second);
    }
    printf("\n==================================================\n");

    const bool ok = verdict && !strncmp(verdict, "SUCCESS", 7);
    return ok ? 0 : 1;
}
