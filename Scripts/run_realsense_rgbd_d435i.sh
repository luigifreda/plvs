#!/usr/bin/env bash

SUFFIX="_old" # comment if you want to use the new example binaries

#export DEBUG_PREFIX="gdb -ex run --args"  # uncomment this in order to debug with gdb
#export DEBUG_PREFIX="valkyrie "

$DEBUG_PREFIX ../Examples$SUFFIX/RGB-D/rgbd_realsense_D435i$SUFFIX ../Vocabulary/ORBvoc.txt ../Examples$SUFFIX/RGB-D/RealSense_D435i_calib.yaml
