#!/usr/bin/env bash


# N.B.:
# 1) you can kill all the spawned terminals together by right clicking on
# the X icon on the left bar and selecting "Quit"

#echo "usage: ./${0##*/} "

SUFFIX="_old" # comment if you want to use the new example binaries

EUROC_YAML="EuRoC.yaml"

SEQUENCE="V102"
#SEQUENCE="V202"
#SEQUENCE="MH02"
#SEQUENCE="MH03"
#SEQUENCE="V203" # difficult 

# possibile sequences
#MH01
#MH02
#MH03
#MH04
#MH05
#V101
#V102
#V103
#V201
#V202
#V203

PATH_TO_SEQUENCE_FOLDER="$HOME/Work/datasets/rgbd_datasets/euroc/$SEQUENCE"
OUTPUT_BASE_NAME="dataset_"$SEQUENCE"_stereo"

echo PATH_TO_SEQUENCE_FOLDER: $PATH_TO_SEQUENCE_FOLDER
echo OUTPUT_BASE_NAME: $OUTPUT_BASE_NAME

#export DEBUG_PREFIX="gdb -ex run --args"  # uncomment this in order to debug with gdb
#export DEBUG_PREFIX="valkyrie "

# activate parallel memory logging
#xterm -e "echo proc info recording; ./get_proc_info.sh ; bash" &

#set -x

$DEBUG_PREFIX ../Examples$SUFFIX/Stereo/stereo_euroc$SUFFIX \
	../Vocabulary/ORBvoc.txt \
	../Examples$SUFFIX/Stereo/$EUROC_YAML \
	$PATH_TO_SEQUENCE_FOLDER \
	../Examples$SUFFIX/Stereo/EuRoC_TimeStamps/$SEQUENCE'.txt' \
	$OUTPUT_BASE_NAME

echo "" 
echo metric generated with evaluate_ate_scale.py: 
python evaluate_ate_scale.py ../evaluation/Ground_truth/EuRoC_left_cam/"$SEQUENCE"_GT.txt "f_"$OUTPUT_BASE_NAME".txt" --plot "f_"$OUTPUT_BASE_NAME".pdf"

echo "" 
echo metrics generated with evaluate_euroc.sh:
./evaluate_euroc.sh $PATH_TO_SEQUENCE_FOLDER $OUTPUT_BASE_NAME".txt"


./move_output_to_results.sh $OUTPUT_BASE_NAME