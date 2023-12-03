#!/bin/bash

# N.B.: 
# 1) you can kill all the spawned terminals together by right clicking on
# the X icon on the left bar and selecting "Quit"

#echo "usage: ./${0##*/} "

SUFFIX="_old" 

INSEG_YAML="InSeg.yaml"

# You can download the datasets here https://rgbd-dataset.cs.washington.edu/dataset/rgbd-scenes-v2/
PATH_TO_SEQUENCE_FOLDER="~/Work/datasets/rgbd_datasets/inseg_dataset"

#export DEBUG_PREFIX="gdb -ex run --args"  # uncomment this in order to debug with gdb
#export DEBUG_PREFIX="valkyrie "

eval PATH_TO_SEQUENCE_FOLDER=$PATH_TO_SEQUENCE_FOLDER
echo $PATH_TO_SEQUENCE_FOLDER 

$DEBUG_PREFIX ../Examples$SUFFIX/RGB-D/rgbd_inseg ../Vocabulary/ORBvoc.txt ../Settings/$INSEG_YAML $PATH_TO_SEQUENCE_FOLDER 


./move_output_to_results.sh
