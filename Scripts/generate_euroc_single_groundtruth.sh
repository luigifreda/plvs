#!/bin/bash

# EXAMPLE$ ./generate_groundtruth.sh euroc $HOME/Work/datasets/rgbd_datasets/euroc/MH03/

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd ) # get script dir 
SCRIPT_DIR=$(readlink -f $SCRIPT_DIR)  # this reads the actual path if a symbolic directory is used
cd $SCRIPT_DIR # this brings us in the actual used folder (not the symbolic one)


TIME_SCALE_FACTOR=1e+9 # used to multiply the timestamps and bring them to TUM timestamp scale 

DATASET_PATH=$1
if [[ -n "$DATASET_PATH" ]]; then
	echo ...
else
    echo "missing input dataset"
    exit 1 
fi 

DATASET_PATH="$DATASET_PATH/mav0/state_groundtruth_estimate0/"
echo DATASET_PATH=$DATASET_PATH

cd $DATASET_PATH
evo_traj euroc data.csv --save_as_tum
# multiply the timestamps for a scale factor
$SCRIPT_DIR/multiply_timestamps.py data.tum data_t2.tum $TIME_SCALE_FACTOR

cd $SCRIPT_DIR
