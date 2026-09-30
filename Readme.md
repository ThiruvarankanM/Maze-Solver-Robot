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
├── include/        # Header files (pin maps, config)
├── lib/            # Modules: motors, sensors, maze, line follower
├── src/            # main.cpp
├── test/           # Unit tests
├── docs/           # Wiring diagrams, design notes
└── platformio.ini
```

## Getting Started

1. Install [PlatformIO](https://platformio.org/) (in VS Code).
2. Clone the repo:
   ```bash
   git clone https://github.com/<your-username>/EE4360-Maze-Solver-Robot.git
   ```
3. Open the folder in PlatformIO, then **Build** and **Upload** to the Arduino Mega.

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

- Work on feature branches (`feature/<name>`) and merge into `main` through pull requests.
- Commit small changes often, with clear messages (e.g. `feat: add PID line follower`).
