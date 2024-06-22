#!/usr/bin/env bash

#echo "usage: ./${0##*/} "

#SUFFIX="_old" # comment if you want to use the new example binaries

DATASET_BASE_FOLDER="$HOME/Work/datasets/tum_vi_dataset" #Example, it is necesary to change it by the dataset path

DATASET_NAME="dataset-magistrale1_512_16"
#DATASET_NAME="dataset-corridor1_512_16"
#DATASET_NAME="dataset-room1_512_16"
#DATASET_NAME="dataset-outdoors8_512_16"  # <-- use the "far" YAML settings 
#DATASET_NAME="dataset-slides3_512_16"

# NOTE: the "rectified" settings allows to use lines and volumetric mapping
TUM_YAML="TUM-VI.yaml"
#TUM_YAML="TUM-VI_far.yaml"  # for outdoor envs 
#TUM_YAML="TUM-VI_rectified.yaml" # use this if you want to pre-rectify the images
#TUM_YAML="TUM-VI_far_rectified.yaml" # use this if you want to pre-rectify the images


#export DEBUG_PREFIX="gdb -ex run --args"  # uncomment this in order to debug with gdb
#export DEBUG_PREFIX="valkyrie "

# Single Session Example

RESULTS_LOG=$DATASET_NAME"_stereo"
DATASET_TIMESTAMPS_DATA=../Examples$SUFFIX/Stereo/TUM_TimeStamps/${DATASET_NAME:0:-3}".txt"  # remove the last 3 chars

echo "Launching with Stereo sensor"
$DEBUG_PREFIX ../Examples$SUFFIX/Stereo/stereo_tum_vi$SUFFIX \
	../Vocabulary/ORBvoc.txt \
	../Examples$SUFFIX/Stereo/$TUM_YAML \
	$DATASET_BASE_FOLDER/$DATASET_NAME"/mav0/cam0/data" $DATASET_BASE_FOLDER/$DATASET_NAME"/mav0/cam1/data" \
	$DATASET_TIMESTAMPS_DATA \
	$RESULTS_LOG

echo "------------------------------------"
echo "Evaluation of $DATASET_NAME trajectory with Stereo sensor"
python evaluate_ate_scale.py $DATASET_BASE_FOLDER/$DATASET_NAME"/mav0/mocap0/data.csv" "f_"$RESULTS_LOG".txt" --plot "f_"$RESULTS_LOG".pdf"


./move_output_to_results.sh $RESULTS_LOG

