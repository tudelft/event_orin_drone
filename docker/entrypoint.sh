#!/bin/bash
set -e

# from https://github.com/dusty-nv/jetson-containers/blob/master/packages/ros/ros_entrypoint.sh
# autocomplete: https://github.com/osrf/docker_images/issues/114
function ros_source_env() 
{
	if [ -f "$1" ]; then
		echo "sourcing   $1"
		source "$1"
		echo "source $1" >> ~/.bashrc
	else
		echo "not found   $1"
	fi	
}

# source local ros install if present
ros_source_env "${ROS_ROOT}/setup.bash"
ros_source_env "install/setup.bash"

# prepend correct realsense libs to LD_LIBRARY_PATH after sourcing ros
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH

# execute the passed command
exec "$@"
