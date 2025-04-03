#!/usr/bin/env bash

# just clean plvs ROS workspaces

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd ) # get script dir 
SCRIPT_DIR=$(readlink -f $SCRIPT_DIR)  # this reads the actual path if a symbolic directory is used
cd $SCRIPT_DIR # this brings us in the actual used folder (not the symbolic one)

echo "cleaning ros ws "
if [[ -d ros_ws ]]; then 
	cd ros_ws
	if [[ -d build ]]; then 
		rm -R build
	fi
	if [[ -d devel ]]; then 
		rm -R devel 
	fi
	if [[ -d install ]]; then 
		rm -R install 
	fi		
	if [[ -d log ]]; then 
		rm -R log 
	fi		
fi 

echo "cleaning ros2 ws "
if [[ -d ros2_ws ]]; then 
	cd ros2_ws
	if [[ -d build ]]; then 
		rm -R build
	fi
	if [[ -d devel ]]; then 
		rm -R devel 
	fi
	if [[ -d install ]]; then 
		rm -R install 
	fi	
	if [[ -d log ]]; then 
		rm -R log 
	fi		
fi 