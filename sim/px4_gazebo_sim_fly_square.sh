#!/usr/bin/env bash

root_dir=$HOME/projects

# px4 and gazebo
tmux new-session -d -s px4_gazebo_sim "cd $root_dir/event_orin_drone/fc/PX4-Autopilot && make px4_sitl gz_x500_depth"
# micro XRCE-DDS agent
tmux split-window "snap run micro-xrce-dds-agent udp4 -p 8888"
# ros-gazebo image bridge for depth image (has to run on host like gazebo)
# tmux split-window "ros2 run ros_gz_image image_bridge /depth_camera /depth_camera"
# fly square node (sleep to prevent timeout)
tmux split-window "sleep 10 && cd $root_dir/event_orin_drone && jetson-containers run -v .:/workspace -w /workspace/ros --user $(id -u):$(id -g) --ipc=host event_orin_drone ros2 run eo_drone fly_square"
# rqt for visualization (inside docker to get px4 msg types)
# tmux split-window "cd $root_dir/ && jetson-containers run -v .:/workspace --user $(id -u):$(id -g) --ipc=host ole_ros rqt"
# show tmux nicely tiled
tmux select-layout tiled
tmux attach -t px4_gazebo_sim
