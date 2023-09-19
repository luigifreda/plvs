#!/bin/bash

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd ) # get script dir 
SCRIPT_DIR=$(readlink -f $SCRIPT_DIR)  # this reads the actual path if a symbolic directory is used
cd $SCRIPT_DIR # this brings us in the actual used folder (not the symbolic one)


ASSOCIATE_SCRIPT="$SCRIPT_DIR/associate.py"


DATASET_BASE_FOLDER="$HOME/Work/datasets/rgbd_datasets/tum"

DATASETS=( \
rgbd_dataset_freiburg1_360 \
rgbd_dataset_freiburg1_desk \
rgbd_dataset_freiburg1_desk2 \
rgbd_dataset_freiburg1_floor \
rgbd_dataset_freiburg1_plant \
rgbd_dataset_freiburg1_room \
rgbd_dataset_freiburg2_rpy \
rgbd_dataset_freiburg1_teddy \
rgbd_dataset_freiburg1_xyz \
rgbd_dataset_freiburg2_360_hemisphere \
rgbd_dataset_freiburg2_360_kidnap \
rgbd_dataset_freiburg2_coke \
rgbd_dataset_freiburg2_desk \
rgbd_dataset_freiburg2_dishes \
rgbd_dataset_freiburg2_large_no_loop \
rgbd_dataset_freiburg2_large_with_loop \
rgbd_dataset_freiburg2_metallic_sphere \
rgbd_dataset_freiburg2_metallic_sphere2 \
rgbd_dataset_freiburg2_pioneer_360 \
rgbd_dataset_freiburg2_pioneer_slam \
rgbd_dataset_freiburg2_pioneer_slam2 \
rgbd_dataset_freiburg2_pioneer_slam3 \
rgbd_dataset_freiburg2_rpy \
rgbd_dataset_freiburg2_xyz \
rgbd_dataset_freiburg3_cabinet \
rgbd_dataset_freiburg3_large_cabinet \
rgbd_dataset_freiburg3_long_office_household \
rgbd_dataset_freiburg3_nostructure_notexture_far \
rgbd_dataset_freiburg3_nostructure_notexture_near_withloop \
rgbd_dataset_freiburg3_nostructure_texture_far \
rgbd_dataset_freiburg3_nostructure_texture_near_withloop \
rgbd_dataset_freiburg3_structure_notexture_far \
rgbd_dataset_freiburg3_structure_notexture_near \
rgbd_dataset_freiburg3_structure_texture_far \
rgbd_dataset_freiburg3_structure_texture_near \
rgbd_dataset_freiburg3_teddy \
)

for dataset in "${DATASETS[@]}"
do
	echo "============================================================="
    echo dataset: $dataset

    RGBD_DATASET_PATH=$DATASET_BASE_FOLDER/$dataset
    echo RGBD_DATASET_PATH=$RGBD_DATASET_PATH

    python $ASSOCIATE_SCRIPT $RGBD_DATASET_PATH/rgb.txt $RGBD_DATASET_PATH/depth.txt > associations.txt
    mv associations.txt $RGBD_DATASET_PATH
done