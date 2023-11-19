#!/usr/bin/env bash

SUFFIX="_old" # comment if you want to use the new example binaries

#export DEBUG_PREFIX="gdb -ex run --args"  # uncomment this in order to debug with gdb
#export DEBUG_PREFIX="valkyrie "

$DEBUG_PREFIX ../Examples$SUFFIX/Monocular-Inertial/mono_inertial_realsense_D435i$SUFFIX ../Vocabulary/ORBvoc.bin ../Examples$SUFFIX/Monocular-Inertial/RealSense_D435i_calib.yaml
