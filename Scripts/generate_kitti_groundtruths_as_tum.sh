#!/bin/bash

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd ) # get script dir 
SCRIPT_DIR=$(readlink -f $SCRIPT_DIR)  # this reads the actual path if a symbolic directory is used
cd $SCRIPT_DIR # this brings us in the actual used folder (not the symbolic one)


KITTI_DATASETS_PATH=$1
if [[ -n "$KITTI_DATASETS_PATH" ]]; then
	echo ...
else
    echo missing datasets base path... using default 
	KITTI_DATASETS_PATH="$HOME/Work/datasets/rgbd_datasets/kitti/dataset/sequences"
fi

echo KITTI_DATASETS_PATH: $KITTI_DATASETS_PATH


DATASETS=( \
00 \
01 \
02 \
03 \
04 \
05 \
06 \
07 \
08 \
09 \
10 \
)

for dataset in "${DATASETS[@]}"
do
	echo "============================================================="
	echo "processing $dataset"
	DATASET_PATH="$KITTI_DATASETS_PATH/$dataset/"

	if [ -d $DATASET_PATH ] ; then
		cd $DATASET_PATH
        POSE_FILE=$dataset".txt"
        TIMESTAMP_FILE="times.txt"
		$SCRIPT_DIR/kitti_poses_and_timestamps_to_trajectory.py $POSE_FILE $TIMESTAMP_FILE gt_tum.txt
	else
		echo "dataset $DATASET_PATH not found"
	fi
done



