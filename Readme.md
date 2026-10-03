# EE4360 Maze Solver Robot

Autonomous maze-solving robot for the **EE4360 Maze Solver Challenge 2026**, Department of Electrical Engineering, University of Moratuwa.

The robot starts in **Section A (4×4)**, finds the bridge, follows a line across it, then solves **Section B (9×9)** to reach the finish tile. After exploring, it runs the combined shortest path as fast as possible.

## Features

- Flood-fill maze exploration and shortest-path optimisation
- Line-following across the bridge with an 8-channel IR array (PID)
- Wall detection with 3 ultrasonic sensors
- Heading correction with an MPU-6050 gyro, and odometry from encoder motors
- Detects the approach marker, bridge, and white start and finish tiles

## Hardware

| Component | Details |
|---|---|
| Microcontroller | Arduino Mega 2560 |
| Motors | 2 × DC encoder motors + H-bridge drivers |
| Sensors | 3 × ultrasonic, 8-channel IR array, MPU-6050 |
| Chassis | Custom 3D-printed (≤ 18.5 × 18.5 × 18 cm) |
| Power | Onboard battery (≤ 15 V) + voltage regulator |

## Project Structure

```
Maze-Solver-Robot/
├── include/
│   ├── pins.h                  # Every pin number
│   └── config.h                # Speeds, thresholds, arena sizes
├── lib/
│   ├── Drive/                  # Motors, encoders, gyro
│   ├── WallSensors/            # Ultrasonic sensors
│   ├── LineSensor/             # IR array
│   ├── LineFollower/           # PID line following
│   ├── Maze/                   # Flood fill + shortest path
│   ├── MazeStore/              # Save maze to EEPROM
│   └── Navigator/              # Position, heading, cell moves
├── src/
│   └── main.cpp                # Run state machine
├── test/
│   └── test_maze/              # Flood fill tests (run on laptop)
├── docs/
│   └── Maze_Solver.pdf         # Project brief
├── .github/
│   ├── workflows/build.yml     # CI: build + tests on every PR
│   └── pull_request_template.md
└── platformio.ini              # Build environments
```

### What goes where

| Folder | Purpose | Rule |
|---|---|---|
| `include/` | Shared settings used by every module | All pin numbers go in `pins.h`, all tunable numbers in `config.h` — never hardcode them elsewhere |
| `lib/` | One folder per module, each with its own `.h` and `.cpp` | A module does one job only |
| `src/main.cpp` | Decides what the robot does next (`CALIBRATE → EXPLORE_A → CROSS_BRIDGE → EXPLORE_B → RETURN_TO_START → FAST_RUN → FINISHED`) | Calls modules, contains no low-level hardware code |
| `test/` | Automatic tests that run on a laptop | Only for code without `Arduino.h` (e.g. `Maze`) |
| `docs/` | Brief, wiring diagram, design notes, demo video links, commit-map screenshots | |
| `.github/` | CI and templates | |

### How the modules connect

```
                 main.cpp  (state machine)
                     │
      ┌──────────────┼──────────────┐
  Navigator        Maze        LineFollower
      │          MazeStore          │
  ┌───┴─────┐                   LineSensor
Drive  WallSensors
```

- **Top:** `main.cpp` picks the current stage and calls the modules for it.
- **Middle:** `Navigator`, `Maze`, `MazeStore` and `LineFollower` make the decisions.
- **Bottom:** `Drive`, `WallSensors` and `LineSensor` talk to the hardware. Only these touch pins.

### Build environments

| Environment | What it does |
|---|---|
| `mega` (default) | The full robot program |
| `native` | Runs `test/` on your laptop |

## Getting Started

1. Install [PlatformIO](https://platformio.org/) (in VS Code).
2. Clone the repo:
   ```bash
   git clone https://github.com/ThiruvarankanM/Maze-Solver-Robot.git
   ```
3. Open the folder in PlatformIO, then **Build** and **Upload** to the Arduino Mega.
4. Run the tests on your laptop: `pio test -e native`.

## Roadmap

- [ ] Motor control and encoder odometry
- [ ] Sensor calibration (ultrasonic, IR, gyro)
- [ ] Section A exploration and bridge detection (Mid Evaluation)
- [ ] Line-following across the bridge
- [ ] Section B navigation and finish detection
- [ ] Path optimisation and fast run (Final Evaluation)

## Team — Group 3

| Name | Index No. |
|---|---|
| Abisan S. | 220013N |
| Changeethan S. | 220084F |
| Sampavi J. | 220561P |
| Thiruvarankan M. | 220647K |

## Workflow

- Merge changes into `main` through pull requests.
- Commit small changes often, with clear messages (e.g. `feat: add PID line follower`).
