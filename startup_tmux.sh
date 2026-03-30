#!/bin/bash

SESSION="laplace_pathfinder"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SETUP="source /opt/ros/humble/setup.bash && source \"$SCRIPT_DIR/install/setup.bash\""

tmux kill-session -t "$SESSION" 2>/dev/null
tmux new-session -d -s "$SESSION"
tmux set-option -t "$SESSION" pane-border-status top
tmux set-option -t "$SESSION" pane-border-format " #{pane_title} "

tmux split-window -v -p 50 -t "$SESSION:0.0"
tmux split-window -h -p 50 -t "$SESSION:0.0"
tmux split-window -h -p 67 -t "$SESSION:0.1"
tmux split-window -h -p 50 -t "$SESSION:0.3"


tmux select-pane -t "$SESSION:0.0" -T "Simulation"
tmux send-keys -t "$SESSION:0.0" "$SETUP && ros2 launch laplace_pathfinder simulation.launch.py" Enter

tmux select-pane -t "$SESSION:0.2" -T "Robot Nav Planner"
tmux send-keys -t "$SESSION:0.2" "$SETUP && ros2 launch laplace_pathfinder robot_nav_planner.launch.py" Enter

tmux select-pane -t "$SESSION:0.1" -T "Robot Nav Controller"
tmux send-keys -t "$SESSION:0.1" "$SETUP && ros2 launch laplace_pathfinder robot_nav_controller.launch.py" Enter

tmux select-pane -t "$SESSION:0.3" -T "Local Costmap"
tmux send-keys -t "$SESSION:0.3" "$SETUP && ros2 launch laplace_pathfinder local_costmap.launch.py" Enter

tmux select-pane -t "$SESSION:0.4" -T "Perturbation"
tmux send-keys -t "$SESSION:0.4" "$SETUP && ros2 launch laplace_pathfinder perturbation.launch.py" Enter

tmux select-pane -t "$SESSION:0.0"
tmux attach-session -t "$SESSION"
