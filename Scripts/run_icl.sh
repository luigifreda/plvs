#!/bin/bash


# N.B.: 
# 1) you can kill all the spawned terminals together by right clicking on
# the X icon on the left bar and selecting "Quit"

#echo "usage: ./${0##*/} "

SUFFIX="_old" 

# from https://www.doc.ic.ac.uk/~ahanda/VaFRIC/iclnuim.html 
DATASET_BASE_FOLDER="$HOME/Work/datasets/rgbd_datasets/icl_nuim/"

DATASET="living_room_traj0_frei_png"
#DATASET="living_room_traj1_frei_png"
#DATASET="living_room_traj2_frei_png"
#DATASET="living_room_traj3_frei_png"
#DATASET="living_room_traj0n_frei_png"
#DATASET="living_room_traj1n_frei_png"
#DATASET="living_room_traj2n_frei_png"
#DATASET="living_room_traj3n_frei_png"
#DATASET="traj0_frei_png"
#DATASET="traj1_frei_png"
#DATASET="traj2_frei_png"
#DATASET="traj3_frei_png"

DATASET_PATH=$DATASET_BASE_FOLDER/$DATASET
echo DATASET_PATH: $DATASET_PATH

#export DEBUG_PREFIX="gdb -ex run --args"  # uncomment this in order to debug with gdb
#export DEBUG_PREFIX="valkyrie "

$DEBUG_PREFIX ../Examples$SUFFIX/RGB-D/rgbd_tum$SUFFIX ../Vocabulary/ORBvoc.txt ../Examples$SUFFIX/RGB-D/ICL.yaml $DATASET_PATH $DATASET_PATH/associations.txt


#./perform_evaluation.sh $DATASET_PATH
./evaluate_icl_nuim.sh $DATASET_PATH 


./move_output_to_results.sh
