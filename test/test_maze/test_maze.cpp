#include <unity.h>

#include "Maze.h"

void setUp() {}
void tearDown() {}

// ===================================================================
// Existing tests - unchanged
// ===================================================================

void test_open_grid_distance()
{
    Maze maze(4);
    maze.floodFill(3, 3);
    TEST_ASSERT_EQUAL_UINT8(6, maze.distance(0, 0));
    TEST_ASSERT_EQUAL_UINT8(0, maze.distance(3, 3));
}

void test_wall_is_shared_by_both_cells()
{
    Maze maze(4);
    maze.setWall(1, 1, EAST);
    TEST_ASSERT_TRUE(maze.hasWall(2, 1, WEST));
}

void test_outer_boundary_is_a_wall()
{
    Maze maze(4);
    TEST_ASSERT_TRUE(maze.hasWall(0, 0, WEST));
    TEST_ASSERT_TRUE(maze.hasWall(3, 3, NORTH));
}

void test_wall_forces_detour()
{
    // Wall between (0,0) and (0,1): must go east first
    Maze maze(2);
    maze.setWall(0, 0, NORTH);
    maze.floodFill(0, 1);
    TEST_ASSERT_EQUAL_UINT8(3, maze.distance(0, 0));

    Dir out = SOUTH;
    TEST_ASSERT_TRUE(maze.bestDirection(0, 0, NORTH, out));
    TEST_ASSERT_EQUAL(EAST, out);
}

// ===================================================================
// New: bounds checking
// ===================================================================

void test_cell_outside_the_maze_reports_a_wall()
{
    // Stops a drifting robot from reading or writing past the arrays.
    Maze maze(4);
    TEST_ASSERT_TRUE(maze.hasWall(4, 0, NORTH));
    TEST_ASSERT_TRUE(maze.hasWall(0, 4, NORTH));
    TEST_ASSERT_TRUE(maze.hasWall(200, 200, EAST));
}

void test_distance_outside_the_maze_is_unreachable()
{
    Maze maze(4);
    maze.floodFill(3, 3);
    TEST_ASSERT_EQUAL_UINT8(Maze::UNREACHABLE, maze.distance(4, 4));
    TEST_ASSERT_EQUAL_UINT8(Maze::UNREACHABLE, maze.distance(255, 0));
}

void test_setting_a_wall_outside_the_maze_is_ignored()
{
    Maze maze(4);
    maze.setWall(9, 9, NORTH);   // must not corrupt anything
    maze.floodFill(3, 3);
    TEST_ASSERT_EQUAL_UINT8(6, maze.distance(0, 0));
}

void test_goal_outside_the_maze_leaves_everything_unreachable()
{
    Maze maze(4);
    maze.floodFill(7, 7);
    TEST_ASSERT_EQUAL_UINT8(Maze::UNREACHABLE, maze.distance(0, 0));
    TEST_ASSERT_EQUAL_UINT8(Maze::UNREACHABLE, maze.distance(3, 3));
}

void test_in_bounds_knows_the_maze_edges()
{
    Maze maze(4);
    TEST_ASSERT_TRUE(maze.inBounds(0, 0));
    TEST_ASSERT_TRUE(maze.inBounds(3, 3));
    TEST_ASSERT_FALSE(maze.inBounds(4, 3));
    TEST_ASSERT_FALSE(maze.inBounds(3, 4));
}

// ===================================================================
// New: walls can be taken away again
// ===================================================================

void test_clearing_a_wall_reopens_it_on_both_sides()
{
    // One bad ultrasonic reading must not block a corridor permanently.
    Maze maze(4);
    maze.setWall(1, 1, EAST);
    TEST_ASSERT_TRUE(maze.hasWall(1, 1, EAST));
    TEST_ASSERT_TRUE(maze.hasWall(2, 1, WEST));

    maze.clearWall(1, 1, EAST);
    TEST_ASSERT_FALSE(maze.hasWall(1, 1, EAST));
    TEST_ASSERT_FALSE(maze.hasWall(2, 1, WEST));
}

void test_clearing_cannot_open_the_outer_boundary()
{
    Maze maze(4);
    maze.clearWall(0, 0, WEST);
    TEST_ASSERT_TRUE(maze.hasWall(0, 0, WEST));
}

void test_clearing_a_wall_restores_the_short_route()
{
    Maze maze(2);
    maze.setWall(0, 0, NORTH);
    maze.floodFill(0, 1);
    TEST_ASSERT_EQUAL_UINT8(3, maze.distance(0, 0));

    maze.clearWall(0, 0, NORTH);
    maze.floodFill(0, 1);
    TEST_ASSERT_EQUAL_UINT8(1, maze.distance(0, 0));
}

// ===================================================================
// New: the safe direction chooser
// REGRESSION TESTS for the confirmed direction-selection bug. The old
// one-result bestDirection(x, y) could not report failure - it handed back
// NORTH even when a wall was there - and has now been removed.
// ===================================================================

void test_safe_direction_fails_when_the_goal_is_unreachable()
{
    Maze maze(4);
    maze.setWall(0, 0, NORTH); // seal the robot into the corner
    maze.setWall(0, 0, EAST);
    maze.floodFill(3, 3);

    Dir out = SOUTH; // sentinel: must be left untouched
    TEST_ASSERT_FALSE(maze.bestDirection(0, 0, NORTH, out));
    TEST_ASSERT_EQUAL(SOUTH, out);
}

void test_safe_direction_never_points_at_a_wall()
{
    Maze maze(4);
    maze.setWall(1, 1, NORTH);
    maze.setWall(1, 1, WEST);
    maze.floodFill(3, 3);

    Dir out = NORTH;
    TEST_ASSERT_TRUE(maze.bestDirection(1, 1, NORTH, out));
    TEST_ASSERT_FALSE(maze.hasWall(1, 1, out));
}

void test_safe_direction_fails_when_already_on_the_goal()
{
    Maze maze(4);
    maze.floodFill(3, 3);
    Dir out = NORTH;
    TEST_ASSERT_FALSE(maze.bestDirection(3, 3, NORTH, out));
}

void test_safe_direction_fails_outside_the_maze()
{
    Maze maze(4);
    maze.floodFill(3, 3);
    Dir out = NORTH;
    TEST_ASSERT_FALSE(maze.bestDirection(9, 9, NORTH, out));
}

void test_safe_direction_prefers_carrying_straight_on()
{
    // Open 4x4, goal (3,3). From (0,0) north and east are both distance 5.
    // Whichever way the robot is already pointing should win, to save a turn.
    Maze maze(4);
    maze.floodFill(3, 3);

    Dir out = SOUTH;
    TEST_ASSERT_TRUE(maze.bestDirection(0, 0, EAST, out));
    TEST_ASSERT_EQUAL(EAST, out);

    out = SOUTH;
    TEST_ASSERT_TRUE(maze.bestDirection(0, 0, NORTH, out));
    TEST_ASSERT_EQUAL(NORTH, out);
}

void test_safe_direction_turns_when_straight_on_is_not_closer()
{
    Maze maze(2);
    maze.setWall(0, 0, NORTH);
    maze.floodFill(0, 1);

    Dir out = SOUTH;
    TEST_ASSERT_TRUE(maze.bestDirection(0, 0, NORTH, out));
    TEST_ASSERT_EQUAL(EAST, out);
}

void test_safe_direction_always_gets_strictly_closer()
{
    Maze maze(4);
    maze.setWall(1, 0, NORTH);
    maze.setWall(2, 2, WEST);
    maze.floodFill(3, 3);

    for (uint8_t x = 0; x < 4; x++) {
        for (uint8_t y = 0; y < 4; y++) {
            Dir out = NORTH;
            if (maze.bestDirection(x, y, NORTH, out)) {
                uint8_t nx, ny;
                TEST_ASSERT_TRUE(maze.neighbour(x, y, out, nx, ny));
                TEST_ASSERT_FALSE(maze.hasWall(x, y, out));
                TEST_ASSERT_TRUE(maze.distance(nx, ny) < maze.distance(x, y));
            }
        }
    }
}

int main()
{
    UNITY_BEGIN();

    RUN_TEST(test_open_grid_distance);
    RUN_TEST(test_wall_is_shared_by_both_cells);
    RUN_TEST(test_outer_boundary_is_a_wall);
    RUN_TEST(test_wall_forces_detour);

    RUN_TEST(test_cell_outside_the_maze_reports_a_wall);
    RUN_TEST(test_distance_outside_the_maze_is_unreachable);
    RUN_TEST(test_setting_a_wall_outside_the_maze_is_ignored);
    RUN_TEST(test_goal_outside_the_maze_leaves_everything_unreachable);
    RUN_TEST(test_in_bounds_knows_the_maze_edges);

    RUN_TEST(test_clearing_a_wall_reopens_it_on_both_sides);
    RUN_TEST(test_clearing_cannot_open_the_outer_boundary);
    RUN_TEST(test_clearing_a_wall_restores_the_short_route);

    RUN_TEST(test_safe_direction_fails_when_the_goal_is_unreachable);
    RUN_TEST(test_safe_direction_never_points_at_a_wall);
    RUN_TEST(test_safe_direction_fails_when_already_on_the_goal);
    RUN_TEST(test_safe_direction_fails_outside_the_maze);
    RUN_TEST(test_safe_direction_prefers_carrying_straight_on);
    RUN_TEST(test_safe_direction_turns_when_straight_on_is_not_closer);
    RUN_TEST(test_safe_direction_always_gets_strictly_closer);

    return UNITY_END();
}
