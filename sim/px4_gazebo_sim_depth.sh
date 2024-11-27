#!/usr/bin/env bash

# px4 and gazebo
tmux new-session -d -s px4_gazebo_sim "cd fc/PX4-Autopilot && PX4_GZ_WORLD=walls make px4_sitl gz_x500_depth"
# micro XRCE-DDS agent
tmux split-window "snap run micro-xrce-dds-agent udp4 -p 8888"
# ros-gazebo image bridge for depth image (has to run on host like gazebo)
tmux split-window "ros2 run ros_gz_image image_bridge /depth_camera /depth_camera"
# show tmux nicely tiled
tmux select-layout tiled
tmux attach -t px4_gazebo_sim
