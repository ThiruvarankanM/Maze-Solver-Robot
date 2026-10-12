# sim

Maze simulator. Runs on a laptop, needs no Arduino.

It links the **real** `lib/Maze` and `lib/Navigator` code that runs on the Mega.
Only the walls, the wall sensors and the movement are faked here, so what you
see is genuinely the algorithm the robot will use.

**It proves nothing about motors, encoders, gyro drift or ultrasonic sensors.**
Those still have to be checked on the robot with `hardware_tests/`.

## Build and run

```bash
pio run -e sim                 # build
.pio/build/sim/program --list  # see the built-in mazes
.pio/build/sim/program --maze obstacles
```

## Options

| Option | Meaning |
|---|---|
| `--list` | show the built-in mazes and exit |
| `--maze NAME` | pick a built-in maze (default `obstacles`) |
| `--file PATH` | load a maze drawing from a text file instead |
| `--start X,Y` | starting cell |
| `--heading N\|E\|S\|W` | starting direction |
| `--goal X,Y` | target cell |
| `--explore` | robot begins with an empty map and senses walls as it goes |
| `--auto` | run straight through without stopping |
| `--delay MS` | frame delay in auto mode (default 350) |
| `--max-steps N` | give up after N moves |

Keys while paused: `Enter` one step, `r` run, `x` reset, `q` quit.
Press `Enter` while it is running to pause again.

## Reading the display

```
+-----+-----+-----+-----+
|S  9     8 |   1  G  0 |     9  = flood fill distance to the goal
+     +     +     +     +     -  = unreachable
|   8     7 |   2     1 |     ^>v< = robot, pointing the way it faces
+     +     +     +     +     G  = goal,  S = start
|   7     6 |   3     2 |
+     +     +     +     +     | and ----- are walls
|   6     5     4     3 |     a gap means the passage is open
+-----+-----+-----+-----+
```

The robot always steps to a neighbour with a **strictly smaller** number, which
is why it can never loop forever. `x` grows EAST (right), `y` grows NORTH (up),
so the top line of a drawing is the highest `y`.

## Writing your own maze

Each cell is five characters wide. A `-` in a horizontal border or a `|` in a
cell row is a wall; a space is an open passage. Save it as a `.txt` file and
pass it with `--file`. A 4x4 needs 9 lines of 25 characters.

```
+-----+-----+-----+-----+
|     |                 |
+     +     +-----+     +
|     |     |           |
+     +     +     +-----+
|           |           |
+-----+     +     +     +
|                       |
+-----+-----+-----+-----+
```
