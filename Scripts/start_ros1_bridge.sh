#!/usr/bin/env bash

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd ) # get script dir
SCRIPT_DIR=$(readlink -f $SCRIPT_DIR)  # this reads the actual path if a symbolic directory is used


if [[ -z ${ROS1_INSTALL_PATH} ]]; then
    source $SCRIPT_DIR/find_ros.sh
fi
if [[ -z ${ROS2_INSTALL_PATH} ]]; then
    source $SCRIPT_DIR/find_ros.sh
fi

if [[ -z ${ROS1_INSTALL_PATH} ]]; then
    echo "ROS1 not found"
    exit 1
fi
if [[ -z ${ROS2_INSTALL_PATH} ]]; then
    echo "ROS2 not found"
    exit 1
fi

source ${ROS1_INSTALL_PATH}/setup.bash
source ${ROS2_INSTALL_PATH}/setup.bash
export ROS_MASTER_URI=http://localhost:11311 

ros2 run ros1_bridge dynamic_bridge --bridge-all-topics
 