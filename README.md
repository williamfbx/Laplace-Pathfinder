# Dynamic Laplace Pathfinder

> ROS 2 + Gazebo workspace for CMU 16-832 Integrated Planning and Learning final course project.

![status](https://img.shields.io/badge/status-template-blue)
![ros2](https://img.shields.io/badge/ROS%202-Humble-informational)
![gazebo](https://img.shields.io/badge/Gazebo-simulation-orange)

---

## Overview

This repository is a **ROS 2 + Gazebo workspace for the `laplace_pathfinder` project** developed for CMU 16-832. It combines:

- **offline map preprocessing** from `.pgm` / `.yaml` inputs
- **Laplace potential field generation** for global guidance
- **online ROS 2 planning and control** for TurtleBot3 navigation
- **dynamic-obstacle perturbation handling** with either classical SOR or learned neural etwork warm-starts

---

## Quick Start

For a class demo or project checkoff, the shortest end-to-end path is:

1. **Build and source the workspace**
   ```bash
   cd ~/16832_ws
   colcon build
   source install/setup.bash
   ```

2. **Generate or copy the map assets** into `src/laplace_pathfinder/maps/`
   - `bookstore.pgm`
   - `bookstore.yaml`

3. **Run the offline preprocessing step**
   ```bash
   python3 src/laplace_pathfinder/offboard/offboard_main.py
   ```

4. **Launch the full simulation + planning stack**
   ```bash
   ros2 launch laplace_pathfinder laplace_pathfinder.launch.py
   ```

5. **Optionally teleop or publish a waypoint**
   ```bash
   export TURTLEBOT3_MODEL=burger
   ros2 run turtlebot3_teleop teleop_keyboard
   ```

---

## Features

- Laplace-potential path planning for TurtleBot3 in ROS 2
- End-to-end Gazebo simulation in bookstore and warehouse worlds
- Scenario-driven start/goal setup via `scenario_config.yaml`
- Offline preprocessing with `offboard_main.py` to generate `*_map.npy` and `*_phi.npy`
- Online planner/controller stack launched through `laplace_pathfinder.launch.py`
- Dynamic obstacle handling with `solver_mode` options: `pure_sor`, `nn_warmstart_sor`, and `nn_only`
- Neural model training and TorchScript export through `train_laplace_nn.py`

---

## Core Components

| Component | File / Package | Purpose |
|---|---|---|
| Simulation launcher | `launch/simulation.launch.py` | Loads the chosen Gazebo world, spawns TurtleBot3, and places start/goal markers |
| Global planner | `robot_nav_planner` | Publishes waypoints by following the precomputed Laplace field |
| Controller | `robot_nav_controller` | Tracks the waypoint and publishes `/cmd_vel` |
| Perturbation node | `perturbation` | Updates the field around new obstacles using SOR and/or the trained CNN |
| Offline solver | `offboard/offboard_main.py` | Converts maps into numpy assets and precomputes the potential field |
| Trainer | `offboard/train_laplace_nn.py` | Trains and exports `models/laplace_nn.pt` for learned perturbation updates |

---

## Repository Structure

```text
16832_ws/
├── src/
│   ├── aws-robomaker-bookstore-world/   # Bookstore simulation world and assets
│   ├── dynamic_logistics_warehouse/     # Warehouse world and assets
│   ├── gazebo_ros2_2Dmap_plugin/        # 2D occupancy map generation plugin
│   └── laplace_pathfinder/
│       ├── config/                      # Planner and navigation parameters
│       ├── data/                        # Training or generated data
│       ├── include/                     # C++ headers
│       ├── launch/                      # ROS 2 launch files
│       ├── maps/                        # Generated map files (`map.pgm`, `map.yaml`)
│       ├── models/                      # Gazebo / robot models
│       ├── offboard/                    # Offline scripts and NN training
│       └── src/                         # Core C++ source files
├── commands.txt                         # Helpful development commands
├── pipeline.txt                         # High-level setup and execution pipeline
└── README.md                            # Project documentation
```

---

## Prerequisites

Before running the project, make sure you have:

- **Ubuntu Linux**
- **ROS 2 Humble** (or a compatible ROS 2 setup already used by this workspace)
- **Gazebo** with `gazebo_ros`
- `colcon` build tools
- TurtleBot3 packages, especially `turtlebot3_gazebo` and `turtlebot3_description`
- Python dependencies for the offboard scripts (`numpy`, `PyYAML`, plotting utilities)
- PyTorch for training or using the learned perturbation model

If you are using the provided ML environment, activate:

```bash
source .venv_torch18/bin/activate
```

CUDA is optional but recommended for the longer `train_laplace_nn.py` runs.

---

## Installation and Build

From the workspace root:

```bash
cd ~/16832_ws
colcon build
source install/setup.bash
```

If you use the training environment:

```bash
source .venv_torch18/bin/activate
```

---

## Setup Pipeline

The following pipeline summarizes the intended setup and execution order from `pipeline.txt`.

### 1) Generate occupancy maps

#### Bookstore world
```bash
cd ~/16832_ws && \
source install/setup.bash && \
export GAZEBO_MODEL_PATH="$GAZEBO_MODEL_PATH:~/16832_ws/src/aws-robomaker-bookstore-world/models" && \
export GAZEBO_RESOURCE_PATH="$GAZEBO_RESOURCE_PATH:~/16832_ws/src/aws-robomaker-bookstore-world" && \
ros2 run gazebo_ros2_2dmap_plugin generate_map.sh \
  ~/16832_ws/src/aws-robomaker-bookstore-world/worlds/bookstore.world \
  ~/16832_ws
```

#### Warehouse world
```bash
cd ~/16832_ws && \
source install/setup.bash && \
export GAZEBO_MODEL_PATH="$GAZEBO_MODEL_PATH:~/16832_ws/src/dynamic_logistics_warehouse/models" && \
export GAZEBO_RESOURCE_PATH="$GAZEBO_RESOURCE_PATH:~/16832_ws/src/dynamic_logistics_warehouse" && \
ros2 run gazebo_ros2_2dmap_plugin generate_map.sh \
  ~/16832_ws/src/dynamic_logistics_warehouse/worlds/warehouse_static.world \
  ~/16832_ws
```

### 2) Place world assets

- Put `dynamic_bookstore.world` in:

```text
src/aws-robomaker-bookstore-world/worlds/
```

### 3) Move generated map files into the package maps directory

After generating the map, place the files in:

```text
src/laplace_pathfinder/maps/
```

For the current offboard script configuration, the filenames should match the `map_stem` used in `src/laplace_pathfinder/offboard/offboard_main.py`.
The repo currently expects the bookstore assets, e.g.:

- `bookstore.pgm`
- `bookstore.yaml`

### 4) Run the offline Laplace preprocessing step

```bash
python3 src/laplace_pathfinder/offboard/offboard_main.py
```

This script:

- loads the occupancy map from `src/laplace_pathfinder/maps/`
- reads the start/goal from `src/laplace_pathfinder/config/scenario_config.yaml`
- saves processed arrays such as `bookstore_map.npy`
- computes and saves the Laplace field such as `bookstore_map_phi.npy`

### 5) (Optional) Train the perturbation neural network

Quick run:

```bash
python src/laplace_pathfinder/offboard/train_laplace_nn.py \
  --epochs 200 \
  --batch-size 32
```

Long training run used in this repo:

```bash
python src/laplace_pathfinder/offboard/train_laplace_nn.py \
  --epochs 3000 \
  --batch-size 8 \
  --base-channels 96 \
  --res-blocks 12 \
  --lr 1e-3 \
  --lr-min 1e-5 \
  --huber-delta 1.0 \
  --weight-alpha 2.0 \
  --weight-cap 5.0 \
  --save-every 100 \
  --device cuda
```

The trained model is exported to:

```text
src/laplace_pathfinder/models/laplace_nn.pt
```

### 6) Check the runtime configuration files

Before launching, verify that these files point to the correct map assets and scenario:

- `src/laplace_pathfinder/config/scenario_config.yaml`
- `src/laplace_pathfinder/config/robot_nav_planner_params.yaml`
- `src/laplace_pathfinder/config/perturbation_params.yaml`

In particular, make sure the map origin, resolution, `phi_file_path`, and `solver_mode` match the map you generated.

### 7) Launch the simulator and planner stack

```bash
ros2 launch laplace_pathfinder laplace_pathfinder.launch.py
```

---

## Common Run Commands

### Launch the worlds

```bash
ros2 launch aws_robomaker_bookstore_world bookstore.launch.py gui:=true
ros2 launch dynamic_logistics_warehouse logistics_warehouse.launch.py
```

### Start the planner stack

```bash
ros2 launch laplace_pathfinder laplace_pathfinder.launch.py
```

### Teleoperate the robot

```bash
export TURTLEBOT3_MODEL=burger
ros2 run turtlebot3_teleop teleop_keyboard
```

### Send a waypoint manually

```bash
ros2 topic pub --once /waypoint geometry_msgs/msg/Point "{x: -5.5, y: 6.5, z: 0.0}"
```

### Start the navigation controller

```bash
ros2 run laplace_pathfinder robot_nav_controller
```

---

## Debugging

Trigger the debug topic:

```bash
ros2 topic pub --once /debug_trigger std_msgs/msg/Empty {}
```

If Gazebo assets are not found during map generation, verify that:

- `install/setup.bash` has been sourced
- `GAZEBO_MODEL_PATH` points to the relevant `models/` directory
- `GAZEBO_RESOURCE_PATH` points to the matching world package root

---

## Generated Artifacts

When the repo is set up correctly, the main outputs live under `src/laplace_pathfinder/`:

- `maps/bookstore_map.npy` — processed binary occupancy map
- `maps/bookstore_map_phi.npy` — precomputed Laplace potential field
- `maps/phi_debug.npy` — planner debug dump
- `models/laplace_nn.pt` — TorchScript perturbation model used by the ROS node
- `data/sample_*/` — optional training samples collected when `data_collection: true`

If you are preparing a report/demo, this is also the right place to add:

- success rate across scenarios
- planning time / controller behavior
- screenshots or videos from Gazebo
- comparison of `pure_sor` vs `nn_warmstart_sor` vs `nn_only`

---

## Known Issues / Notes

- Map generation is sensitive to `GAZEBO_MODEL_PATH` and `GAZEBO_RESOURCE_PATH`.
- `dynamic_bookstore.world` must exist in `src/aws-robomaker-bookstore-world/worlds/` for the dynamic bookstore setup.
- `offboard_main.py` currently uses a hardcoded `map_stem` (`bookstore`), so change that when switching environments.
- Training with `--device cuda` requires a working CUDA/PyTorch installation.

---

## Team / Authors

Package metadata currently lists:

- **Boxiang Fu** (`boxiangf@andrew.cmu.edu`)

Add additional teammates/contributors here if needed.

---

## License

Per `package.xml`, this project uses:

```text
Apache License 2.0
```

---

## Acknowledgments

- CMU 16-832 course staff
- ROS 2 and Gazebo maintainers
- AWS RoboMaker world assets contributors
