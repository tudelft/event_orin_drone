#!/usr/bin/env bash

root_dir=$HOME/projects

# mavlink-router
tmux new-session -d -s drone_flight "mavlink-routerd"
# micro XRCE-DDS agent
tmux split-window "micro-xrce-dds-agent serial --dev /dev/ttyTHS3 -b 3000000"
# empty container
tmux split-window "cd $root_dir/event_orin_drone && jetson-containers run -v .:/workspace --user $(id -u):$(id -g) event_orin_drone"
# htop (jtop not working?)
tmux split-window "htop"
# show tmux nicely tiled
tmux select-layout tiled
tmux attach -t drone_flight
