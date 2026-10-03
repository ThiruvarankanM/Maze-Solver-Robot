#include <unity.h>

#include "Maze.h"

void setUp() {}
void tearDown() {}

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
    TEST_ASSERT_EQUAL(EAST, maze.bestDirection(0, 0));
}

int main()
{
    UNITY_BEGIN();
    RUN_TEST(test_open_grid_distance);
    RUN_TEST(test_wall_is_shared_by_both_cells);
    RUN_TEST(test_outer_boundary_is_a_wall);
    RUN_TEST(test_wall_forces_detour);
    return UNITY_END();
}
