#!/usr/bin/env bash


# N.B.:
# 1) you can kill all the spawned terminals together by right clicking on
# the X icon on the left bar and selecting "Quit"

#echo "usage: ./${0##*/} "

SUFFIX="_old" 

PATH_TO_SEQUENCE_FOLDER="$HOME/Work/datasets/rgbd_datasets/kitti/dataset/sequences"
SEQUENCE_NUMBER=6
#SEQUENCE_NUMBER=4


if [ $SEQUENCE_NUMBER -lt 10 ];then
	SEQUENCE="0$SEQUENCE_NUMBER"
else
	SEQUENCE="$SEQUENCE_NUMBER"
fi

echo "sequence: $SEQUENCE"

KITTI_YAML="KITTI04-12.yaml" # default
if [ "$SEQUENCE_NUMBER" -le 2 ];then
	KITTI_YAML="KITTI00-02.yaml"
else
	if [ "$SEQUENCE_NUMBER" -eq 3 ];then
		KITTI_YAML="KITTI03.yaml"
	fi
fi


eval PATH_TO_SEQUENCE_FOLDER=$PATH_TO_SEQUENCE_FOLDER
echo $PATH_TO_SEQUENCE_FOLDER


#export DEBUG_PREFIX="gdb -ex run --args"  # uncomment this in order to debug with gdb
#export DEBUG_PREFIX="valkyrie "

# activate parallel memory logging
#xterm -e "echo proc info recording; ./get_proc_info.sh ; bash" &


$DEBUG_PREFIX ../Examples$SUFFIX/Monocular/mono_kitti$SUFFIX ../Vocabulary/ORBvoc.txt ../Examples$SUFFIX/Monocular/$KITTI_YAML $PATH_TO_SEQUENCE_FOLDER/$SEQUENCE


#./perform_evaluation.sh $PATH_TO_SEQUENCE_FOLDER
#evo_ape kitti CameraTrajectory.txt $PATH_TO_SEQUENCE_FOLDER/$SEQUENCE/$SEQUENCE.txt -as
./evaluate_kitti.sh $PATH_TO_SEQUENCE_FOLDER/$SEQUENCE/$SEQUENCE.txt 1  # the last 1 is to enable monocular evaluation 


./move_output_to_results.sh