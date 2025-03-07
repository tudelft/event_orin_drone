#!/usr/bin/env bash

jetson-containers run -v .:/workspace -w /workspace/ros --user $(id -u):$(id -g) event_orin_drone colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
