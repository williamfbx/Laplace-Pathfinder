#!/bin/bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source /opt/ros/humble/setup.bash
source "$SCRIPT_DIR/install/setup.bash"

gnome-terminal --title="Simulation" -- bash -c "
  source /opt/ros/humble/setup.bash
  source '$SCRIPT_DIR/install/setup.bash'
  ros2 launch laplace_pathfinder simulation.launch.py
  exec bash
"

gnome-terminal --title="Robot Nav Planner" -- bash -c "
  source /opt/ros/humble/setup.bash
  source '$SCRIPT_DIR/install/setup.bash'
  ros2 launch laplace_pathfinder robot_nav_planner.launch.py
  exec bash
"

gnome-terminal --title="Robot Nav Controller" -- bash -c "
  source /opt/ros/humble/setup.bash
  source '$SCRIPT_DIR/install/setup.bash'
  ros2 launch laplace_pathfinder robot_nav_controller.launch.py
  exec bash
"

gnome-terminal --title="Local Costmap" -- bash -c "
  source /opt/ros/humble/setup.bash
  source '$SCRIPT_DIR/install/setup.bash'
  ros2 launch laplace_pathfinder local_costmap.launch.py
  exec bash
"

gnome-terminal --title="Perturbation" -- bash -c "
  source /opt/ros/humble/setup.bash
  source '$SCRIPT_DIR/install/setup.bash'
  ros2 launch laplace_pathfinder perturbation.launch.py
  exec bash
"
