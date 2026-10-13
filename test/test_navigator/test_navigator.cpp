#include <unity.h>

#include "Maze.h"
#include "Navigator.h"

void setUp() {}
void tearDown() {}

// ------------------------------------------------------------------
// helpers
// ------------------------------------------------------------------

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

// Drives the navigator until it arrives or fails, and checks the two safety
// rules on every single move: it never crosses a wall of the REAL maze, and it
// never ends up outside the maze. Always terminates.
static Navigator::Result drive(const Maze &truth, Navigator &nav, uint8_t gx, uint8_t gy,
                               int &steps, bool explore)
{
    steps = 0;
    const int limit = truth.size() * truth.size() * 8;

    while (steps <= limit) {
        if (explore) {
            const Dir h = nav.heading();
            nav.recordWalls(truth.hasWall(nav.x(), nav.y(), Navigator::leftOf(h)),
                            truth.hasWall(nav.x(), nav.y(), h),
                            truth.hasWall(nav.x(), nav.y(), Navigator::rightOf(h)));
        }

        const uint8_t px = nav.x();
        const uint8_t py = nav.y();
        Dir chosen = NORTH;
        const Navigator::Result r = nav.step(gx, gy, chosen);

        if (r == Navigator::Result::Turned) {
            // Turned on the spot to face a direction the sensors had not
            // covered. Nothing moved, so go round again - the loop above takes
            // the fresh reading that makes the next decision safe.
            TEST_ASSERT_EQUAL_UINT8(px, nav.x());
            TEST_ASSERT_EQUAL_UINT8(py, nav.y());
            TEST_ASSERT_EQUAL(chosen, nav.heading());
            steps++;
            continue;
        }

        if (r != Navigator::Result::Moved) {
            return r;
        }

        TEST_ASSERT_FALSE(truth.hasWall(px, py, chosen));   // never walked through a wall
        TEST_ASSERT_TRUE(truth.inBounds(nav.x(), nav.y())); // never left the maze
        steps++;
    }
    return Navigator::Result::Blocked; // ran out of steps
}

// The "obstacles" maze: a barrier between x=1 and x=2 on the top three rows,
// so the only way across is along the bottom row.
static void buildObstacles(Maze &m)
{
    m.clear();
    for (uint8_t y = 1; y < 4; y++) {
        m.setWall(1, y, EAST);
    }
}

// The "deadend" maze: the bottom row is a corridor sealed from above.
static void buildDeadEnd(Maze &m)
{
    m.clear();
    for (uint8_t x = 1; x < 4; x++) {
        m.setWall(x, 0, NORTH);
    }
}

// ------------------------------------------------------------------
// pose tracking
// ------------------------------------------------------------------

void test_navigator_starts_where_it_was_put()
{
    Maze map(4);
    Navigator nav(map, 1, 2, EAST);
    TEST_ASSERT_EQUAL_UINT8(1, nav.x());
    TEST_ASSERT_EQUAL_UINT8(2, nav.y());
    TEST_ASSERT_EQUAL(EAST, nav.heading());
    TEST_ASSERT_EQUAL_UINT16(0, nav.moveCount());
}

void test_moving_updates_position_and_heading()
{
    Maze map(4);
    Navigator nav(map, 0, 0, SOUTH);
    Dir chosen = NORTH;

    TEST_ASSERT_EQUAL(Navigator::Result::Moved, nav.step(0, 1, chosen));
    TEST_ASSERT_EQUAL(NORTH, chosen);
    TEST_ASSERT_EQUAL_UINT8(0, nav.x());
    TEST_ASSERT_EQUAL_UINT8(1, nav.y());
    TEST_ASSERT_EQUAL(NORTH, nav.heading()); // it turned to face the way it went
    TEST_ASSERT_EQUAL_UINT16(1, nav.moveCount());
}

void test_walls_are_recorded_in_the_robots_own_frame()
{
    // Facing EAST at (1,1): the robot's left is NORTH, front is EAST,
    // right is SOUTH.
    Maze map(4);
    Navigator nav(map, 1, 1, EAST);
    nav.recordWalls(true, false, true);

    TEST_ASSERT_TRUE(map.hasWall(1, 1, NORTH));
    TEST_ASSERT_FALSE(map.hasWall(1, 1, EAST));
    TEST_ASSERT_TRUE(map.hasWall(1, 1, SOUTH));
}

void test_a_wall_seen_once_can_be_unseen()
{
    Maze map(4);
    Navigator nav(map, 1, 1, NORTH);
    nav.recordWalls(false, true, false);
    TEST_ASSERT_TRUE(map.hasWall(1, 1, NORTH));

    nav.recordWalls(false, false, false); // the first reading was wrong
    TEST_ASSERT_FALSE(map.hasWall(1, 1, NORTH));
}

// ------------------------------------------------------------------
// solving, with a fully known map
// ------------------------------------------------------------------

void test_reaches_the_goal_in_an_open_maze()
{
    Maze truth(4);
    Maze belief(4);
    copyWalls(truth, belief);
    Navigator nav(belief, 0, 0, NORTH);

    int steps = 0;
    TEST_ASSERT_EQUAL(Navigator::Result::Arrived, drive(truth, nav, 3, 3, steps, false));
    TEST_ASSERT_EQUAL_UINT8(3, nav.x());
    TEST_ASSERT_EQUAL_UINT8(3, nav.y());
    TEST_ASSERT_EQUAL_UINT16(6, nav.moveCount()); // the true shortest path
}

void test_takes_the_detour_when_a_barrier_is_in_the_way()
{
    Maze truth(4);
    buildObstacles(truth);
    Maze belief(4);
    copyWalls(truth, belief);
    Navigator nav(belief, 0, 3, SOUTH);

    int steps = 0;
    TEST_ASSERT_EQUAL(Navigator::Result::Arrived, drive(truth, nav, 3, 3, steps, false));
    TEST_ASSERT_EQUAL_UINT16(9, nav.moveCount()); // 3 straight across would be 3
}

void test_backs_out_of_a_dead_end()
{
    Maze truth(4);
    buildDeadEnd(truth);
    Maze belief(4);
    copyWalls(truth, belief);
    Navigator nav(belief, 3, 0, WEST); // starts deep inside the dead-end corridor

    int steps = 0;
    TEST_ASSERT_EQUAL(Navigator::Result::Arrived, drive(truth, nav, 0, 3, steps, false));
    TEST_ASSERT_EQUAL_UINT16(6, nav.moveCount());
}

void test_arrives_immediately_when_it_starts_on_the_goal()
{
    Maze map(4);
    Navigator nav(map, 2, 2, NORTH);
    Dir chosen = NORTH;
    TEST_ASSERT_EQUAL(Navigator::Result::Arrived, nav.step(2, 2, chosen));
    TEST_ASSERT_EQUAL_UINT16(0, nav.moveCount());
}

// ------------------------------------------------------------------
// failing safely
// ------------------------------------------------------------------

void test_reports_unreachable_instead_of_driving_into_a_wall()
{
    // REGRESSION: the old chooser returned NORTH here, straight into a wall.
    Maze truth(4);
    truth.setWall(3, 3, SOUTH);
    truth.setWall(3, 3, WEST);
    Maze belief(4);
    copyWalls(truth, belief);
    Navigator nav(belief, 0, 0, NORTH);

    int steps = 0;
    TEST_ASSERT_EQUAL(Navigator::Result::Unreachable,
                      drive(truth, nav, 3, 3, steps, false));
    TEST_ASSERT_EQUAL_UINT16(0, nav.moveCount()); // it never moved at all
}

void test_reports_an_invalid_goal()
{
    Maze map(4);
    Navigator nav(map, 0, 0, NORTH);
    Dir chosen = NORTH;
    TEST_ASSERT_EQUAL(Navigator::Result::InvalidGoal, nav.step(4, 0, chosen));
    TEST_ASSERT_EQUAL(Navigator::Result::InvalidGoal, nav.step(0, 9, chosen));
}

void test_reports_an_invalid_starting_position()
{
    Maze map(4);
    Navigator nav(map, 7, 7, NORTH);
    Dir chosen = NORTH;
    TEST_ASSERT_EQUAL(Navigator::Result::InvalidGoal, nav.step(3, 3, chosen));
}

void test_a_sealed_robot_stops_instead_of_looping_forever()
{
    Maze truth(4);
    truth.setWall(0, 0, NORTH);
    truth.setWall(0, 0, EAST);
    Maze belief(4);
    copyWalls(truth, belief);
    Navigator nav(belief, 0, 0, NORTH);

    int steps = 0;
    TEST_ASSERT_EQUAL(Navigator::Result::Unreachable,
                      drive(truth, nav, 3, 3, steps, false));
    TEST_ASSERT_TRUE(steps < 16); // terminated quickly, did not spin
}

// ------------------------------------------------------------------
// solving while discovering the walls
// ------------------------------------------------------------------

void test_solves_a_maze_it_has_to_discover()
{
    Maze truth(4);
    buildObstacles(truth);
    Maze belief(4); // knows nothing but the outer boundary
    Navigator nav(belief, 0, 3, SOUTH);

    int steps = 0;
    TEST_ASSERT_EQUAL(Navigator::Result::Arrived, drive(truth, nav, 3, 3, steps, true));
    TEST_ASSERT_EQUAL_UINT8(3, nav.x());
    TEST_ASSERT_EQUAL_UINT8(3, nav.y());
}

void test_exploring_a_sealed_goal_still_terminates()
{
    Maze truth(4);
    truth.setWall(3, 3, SOUTH);
    truth.setWall(3, 3, WEST);
    Maze belief(4);
    Navigator nav(belief, 0, 0, NORTH);

    int steps = 0;
    const Navigator::Result r = drive(truth, nav, 3, 3, steps, true);
    TEST_ASSERT_TRUE(r != Navigator::Result::Moved);
    TEST_ASSERT_TRUE(nav.x() != 3 || nav.y() != 3); // it never got there
}

// ------------------------------------------------------------------
// a wall behind the robot
// ------------------------------------------------------------------

// REGRESSION: the robot senses left/front/right only, so a wall BEHIND it is
// unknown and an empty map reads it as open. It used to reverse straight
// through that wall. Now it must turn to face the direction, re-read the front
// sensor, discover the wall and go round.
void test_never_moves_backwards_through_an_unsensed_wall()
{
    Maze truth(3);
    truth.setWall(1, 1, WEST); // directly behind a robot that faces EAST
    Maze belief(3);            // exploring: knows only the outer boundary
    Navigator nav(belief, 1, 1, EAST);

    // drive() asserts on every move that no wall of the REAL maze was crossed.
    int steps = 0;
    TEST_ASSERT_EQUAL(Navigator::Result::Arrived, drive(truth, nav, 0, 1, steps, true));
    TEST_ASSERT_EQUAL_UINT8(0, nav.x());
    TEST_ASSERT_EQUAL_UINT8(1, nav.y());

    // Round the top - (1,1) -> (1,2) -> (0,2) -> (0,1) - not through the wall,
    // which would have been a single move.
    TEST_ASSERT_EQUAL_UINT16(3, nav.moveCount());
    TEST_ASSERT_TRUE(belief.hasWall(1, 1, WEST)); // it learned the wall by turning
}

// A selected direction is NOT a performed move: the turn must be reported on
// its own, and must leave the robot in the same cell.
void test_turning_to_check_a_wall_is_not_a_move()
{
    Maze truth(3);
    truth.setWall(1, 1, WEST);
    Maze belief(3);
    Navigator nav(belief, 1, 1, EAST);

    const Dir h = nav.heading();
    nav.recordWalls(truth.hasWall(1, 1, Navigator::leftOf(h)), truth.hasWall(1, 1, h),
                    truth.hasWall(1, 1, Navigator::rightOf(h)));

    Dir chosen = NORTH;
    TEST_ASSERT_EQUAL(Navigator::Result::Turned, nav.step(0, 1, chosen));
    TEST_ASSERT_EQUAL(WEST, chosen);
    TEST_ASSERT_EQUAL(WEST, nav.heading()); // now facing the way it wants to go
    TEST_ASSERT_EQUAL_UINT8(1, nav.x());    // but it has NOT moved
    TEST_ASSERT_EQUAL_UINT8(1, nav.y());
    TEST_ASSERT_EQUAL_UINT16(0, nav.moveCount());
}

// ------------------------------------------------------------------
// the sensing guard: once a caller senses, it must keep sensing
// ------------------------------------------------------------------

// Senses once at (1,1) facing EAST, which is the state that produces a Turned.
static void senseHere(const Maze &truth, Navigator &nav)
{
    const Dir h = nav.heading();
    nav.recordWalls(truth.hasWall(nav.x(), nav.y(), Navigator::leftOf(h)),
                    truth.hasWall(nav.x(), nav.y(), h),
                    truth.hasWall(nav.x(), nav.y(), Navigator::rightOf(h)));
}

void test_step_after_turning_without_sensing_refuses_to_move()
{
    Maze truth(3);
    truth.setWall(1, 1, WEST);
    Maze belief(3);
    Navigator nav(belief, 1, 1, EAST);

    senseHere(truth, nav);
    Dir chosen = NORTH;
    TEST_ASSERT_EQUAL(Navigator::Result::Turned, nav.step(0, 1, chosen));
    TEST_ASSERT_EQUAL(WEST, nav.heading());

    // No recordWalls() this time. The three readings belong to the old heading,
    // so moving would mean trusting the stale 'WEST is open' in the map.
    TEST_ASSERT_EQUAL(Navigator::Result::NeedsSensing, nav.step(0, 1, chosen));
    TEST_ASSERT_EQUAL_UINT8(1, nav.x()); // did not move
    TEST_ASSERT_EQUAL_UINT8(1, nav.y());
    TEST_ASSERT_EQUAL(WEST, nav.heading()); // and did not turn again
    TEST_ASSERT_EQUAL_UINT16(0, nav.moveCount());

    // Asking again must not quietly give in and move.
    TEST_ASSERT_EQUAL(Navigator::Result::NeedsSensing, nav.step(0, 1, chosen));
    TEST_ASSERT_EQUAL_UINT8(1, nav.x());
    TEST_ASSERT_EQUAL_UINT16(0, nav.moveCount());
}

void test_fresh_readings_let_navigation_continue()
{
    Maze truth(3);
    truth.setWall(1, 1, WEST);
    Maze belief(3);
    Navigator nav(belief, 1, 1, EAST);

    senseHere(truth, nav);
    Dir chosen = NORTH;
    TEST_ASSERT_EQUAL(Navigator::Result::Turned, nav.step(0, 1, chosen));
    TEST_ASSERT_EQUAL(Navigator::Result::NeedsSensing, nav.step(0, 1, chosen));

    // Now give it the reading it asked for: facing WEST, the front sensor sees
    // the wall that was behind the robot all along.
    senseHere(truth, nav);
    TEST_ASSERT_TRUE(belief.hasWall(1, 1, WEST));

    // With the wall on the map it reroutes instead of driving into it.
    TEST_ASSERT_EQUAL(Navigator::Result::Moved, nav.step(0, 1, chosen));
    TEST_ASSERT_EQUAL(NORTH, chosen);
    TEST_ASSERT_EQUAL_UINT8(1, nav.x());
    TEST_ASSERT_EQUAL_UINT8(2, nav.y());
    TEST_ASSERT_EQUAL_UINT16(1, nav.moveCount());
}

// A caller with a fully known map never calls recordWalls(), so the guard must
// stay completely out of its way.
void test_known_map_caller_moves_without_ever_sensing()
{
    Maze map(4); // open, and nothing has ever been sensed
    Navigator nav(map, 0, 0, NORTH);

    Dir chosen = SOUTH;
    TEST_ASSERT_EQUAL(Navigator::Result::Moved, nav.step(0, 1, chosen));
    TEST_ASSERT_EQUAL(NORTH, chosen);
    TEST_ASSERT_EQUAL_UINT8(1, nav.y());
    TEST_ASSERT_EQUAL_UINT16(1, nav.moveCount());
}

void test_known_map_navigation_is_unchanged_by_the_sensing_guard()
{
    // Same maze and the same expected move count as
    // test_takes_the_detour_when_a_barrier_is_in_the_way, driven with the map
    // fully known and recordWalls() never called.
    Maze truth(4);
    buildObstacles(truth);
    Maze belief(4);
    copyWalls(truth, belief);
    Navigator nav(belief, 0, 3, SOUTH);

    int steps = 0;
    // drive() returns any non-Moved result, so a stray NeedsSensing or Turned
    // would fail this assertion rather than pass unnoticed.
    TEST_ASSERT_EQUAL(Navigator::Result::Arrived, drive(truth, nav, 3, 3, steps, false));
    TEST_ASSERT_EQUAL_UINT16(9, nav.moveCount());
}

int main()
{
    UNITY_BEGIN();

    RUN_TEST(test_navigator_starts_where_it_was_put);
    RUN_TEST(test_moving_updates_position_and_heading);
    RUN_TEST(test_walls_are_recorded_in_the_robots_own_frame);
    RUN_TEST(test_a_wall_seen_once_can_be_unseen);

    RUN_TEST(test_reaches_the_goal_in_an_open_maze);
    RUN_TEST(test_takes_the_detour_when_a_barrier_is_in_the_way);
    RUN_TEST(test_backs_out_of_a_dead_end);
    RUN_TEST(test_arrives_immediately_when_it_starts_on_the_goal);

    RUN_TEST(test_reports_unreachable_instead_of_driving_into_a_wall);
    RUN_TEST(test_reports_an_invalid_goal);
    RUN_TEST(test_reports_an_invalid_starting_position);
    RUN_TEST(test_a_sealed_robot_stops_instead_of_looping_forever);

    RUN_TEST(test_solves_a_maze_it_has_to_discover);
    RUN_TEST(test_exploring_a_sealed_goal_still_terminates);

    RUN_TEST(test_never_moves_backwards_through_an_unsensed_wall);
    RUN_TEST(test_turning_to_check_a_wall_is_not_a_move);

    RUN_TEST(test_step_after_turning_without_sensing_refuses_to_move);
    RUN_TEST(test_fresh_readings_let_navigation_continue);
    RUN_TEST(test_known_map_caller_moves_without_ever_sensing);
    RUN_TEST(test_known_map_navigation_is_unchanged_by_the_sensing_guard);

    return UNITY_END();
}
