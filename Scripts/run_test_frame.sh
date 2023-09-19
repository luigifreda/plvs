#!/bin/bash

# N.B.: 
# 1) you can kill all the spawned terminals together by right clicking on
# the X icon on the left bar and selecting "Quit"

#echo "usage: ./${0##*/} "

DATASET_BASE_FOLDER="$HOME/Work/datasets/rgbd_datasets/tum/"

DATASET_BASE_FOLDER="$HOME/Work/datasets/rgbd_datasets/tum/"

# TUM1
#DATASET="rgbd_dataset_freiburg1_360"
#DATASET="rgbd_dataset_freiburg1_desk"
#DATASET="rgbd_dataset_freiburg1_desk2"
#DATASET="rgbd_dataset_freiburg1_floor"
#DATASET="rgbd_dataset_freiburg1_plant"
DATASET="rgbd_dataset_freiburg1_room" # <--- 
#DATASET="rgbd_dataset_freiburg2_rpy"
#DATASET="rgbd_dataset_freiburg1_teddy"
#DATASET="rgbd_dataset_freiburg1_xyz"

# TUM2
#DATASET="rgbd_dataset_freiburg2_360_hemisphere"
#DATASET="rgbd_dataset_freiburg2_360_kidnap"
#DATASET="rgbd_dataset_freiburg2_coke"
#DATASET="rgbd_dataset_freiburg2_desk" # <--- 
#DATASET="rgbd_dataset_freiburg2_dishes"
#DATASET="rgbd_dataset_freiburg2_large_no_loop"
#DATASET="rgbd_dataset_freiburg2_large_with_loop"
#DATASET="rgbd_dataset_freiburg2_metallic_sphere"
#DATASET="rgbd_dataset_freiburg2_metallic_sphere2"
#DATASET="rgbd_dataset_freiburg2_pioneer_360"
#DATASET="rgbd_dataset_freiburg2_pioneer_slam"
#DATASET="rgbd_dataset_freiburg2_pioneer_slam2"
#DATASET="rgbd_dataset_freiburg2_pioneer_slam3"
#DATASET="rgbd_dataset_freiburg2_rpy"
#DATASET="rgbd_dataset_freiburg2_xyz"

# TUM3 
#DATASET="rgbd_dataset_freiburg3_cabinet"
#DATASET="rgbd_dataset_freiburg3_large_cabinet"
#DATASET="rgbd_dataset_freiburg3_long_office_household" # <--- 
#DATASET="rgbd_dataset_freiburg3_nostructure_notexture_far"
#DATASET="rgbd_dataset_freiburg3_nostructure_notexture_near_withloop"
#DATASET="rgbd_dataset_freiburg3_nostructure_texture_far"
#DATASET="rgbd_dataset_freiburg3_nostructure_texture_near_withloop"
#DATASET="rgbd_dataset_freiburg3_structure_notexture_far"
#DATASET="rgbd_dataset_freiburg3_structure_notexture_near"
#DATASET="rgbd_dataset_freiburg3_structure_texture_far"
#DATASET="rgbd_dataset_freiburg3_structure_texture_near"
#DATASET="rgbd_dataset_freiburg3_teddy"


TUM_YAML=""
if [[ $DATASET == *freiburg1* ]]; then
	echo "using TUM1 settings"
	TUM_YAML="TUM1.yaml"
fi
if [[ $DATASET == *freiburg2* ]]; then
	echo "using TUM2 settings"
	TUM_YAML="TUM2.yaml"
fi
if [[ $DATASET == *freiburg3* ]]; then
	echo "using TUM3 settings"
	TUM_YAML="TUM3.yaml"
fi

DATASET_PATH=$DATASET_BASE_FOLDER/$DATASET
echo DATASET_PATH: $DATASET_PATH

#export DEBUG_PREFIX="gdb -ex run --args"  # uncomment this in order to debug with gdb
#export DEBUG_PREFIX="valkyrie "

$DEBUG_PREFIX  ../test/test_frame ../Examples_old/RGB-D/$TUM_YAML $DATASET_PATH $DATASET_PATH/associations.txt




