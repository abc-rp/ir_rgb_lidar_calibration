#!/bin/bash

# Check if tmux is installed, if not, prompt the user to install it
if ! command -v tmux &> /dev/null
then
    echo "tmux could not be found, please install it using: sudo apt-get install tmux"
    exit 1
fi

# Start a new tmux session named "calibration"
tmux new-session -d -s calibration

# Pane 0: T0 - Feature Collection
tmux send-keys -t calibration "source /opt/ros/noetic/setup.bash" C-m
tmux send-keys -t calibration "source /var/home/dan/catkin_ws/devel/setup.bash" C-m
tmux send-keys -t calibration "roslaunch lvt2calib pattern_collection_lc.launch ns_lv:=livox_horizon ns_cv:=rgb" C-m

# Split pane vertically for T1
tmux split-window -v -t calibration

# Pane 1: T1 - LiDAR Pattern Detection
tmux send-keys -t calibration "source /opt/ros/noetic/setup.bash" C-m
tmux send-keys -t calibration "source /var/home/dan/catkin_ws/devel/setup.bash" C-m
tmux send-keys -t calibration "roslaunch lvt2calib livox_horizon_pattern.launch cloud_tp:=/livox/lidar ns_:=livox_horizon" C-m

# Split pane horizontally for T2
tmux split-window -h -t calibration

# Pane 2: T2 - Camera Pattern Detection
tmux send-keys -t calibration "source /opt/ros/noetic/setup.bash" C-m
tmux send-keys -t calibration "source /var/home/dan/catkin_ws/devel/setup.bash" C-m
tmux send-keys -t calibration "roslaunch lvt2calib rgb_cam_pattern.launch image_tp:=/rgb_cam/image_raw ns_:=rgb" C-m

# Select the layout even-horizontal
tmux select-layout even-horizontal

# Attach to the tmux session
tmux attach -t calibration
