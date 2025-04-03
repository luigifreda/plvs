#!/usr/bin/env bash

# just clean plvs and ROS workspaces

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd ) # get script dir 
SCRIPT_DIR=$(readlink -f $SCRIPT_DIR)  # this reads the actual path if a symbolic directory is used
cd $SCRIPT_DIR # this brings us in the actual used folder (not the symbolic one)


echo "cleaning PLVS ..."
if [[ -d build ]]; then 
	cd build 
	make clean    # for removing all the bins (deployed outside build)
	cd $SCRIPT_DIR
	rm -R build lib
fi 

echo "cleaning ros ws "
$SCRIPT_DIR/clean_plvs_ros.sh

cd $SCRIPT_DIR
