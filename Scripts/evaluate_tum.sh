#!/usr/bin/env bash

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd ) # get script dir 
SCRIPT_DIR=$(readlink -f $SCRIPT_DIR)  # this reads the actual path if a symbolic directory is used
cd $SCRIPT_DIR # this brings us in the actual used folder (not the symbolic one)


EVALUATE_SCRIPT="$SCRIPT_DIR/evaluate_ate.py"
TRAJECTORY_PATH=$SCRIPT_DIR
DATASET_PATH=$1
PLOT=$2

DATASET_PATH=$1
if [[ -n "$DATASET_PATH" ]] 
then
	echo "..."
else
	echo missing dataset path...
	exit 1
fi

if [[ -n "$PLOT" ]]
then
	echo "..."
else
    PLOT=0 # default 
fi

eval EVALUATE_SCRIPT=$EVALUATE_SCRIPT
eval TRAJECTORY_PATH=$TRAJECTORY_PATH
eval DATASET_PATH=$DATASET_PATH
eval GROUDTRUTH_DATA_PATH=$DATASET_PATH/groundtruth.txt

echo "DATASET_PATH: $DATASET_PATH"

python3 $EVALUATE_SCRIPT --verbose $GROUDTRUTH_DATA_PATH $TRAJECTORY_PATH/CameraTrajectory.txt --save_transformation transformation.txt --plot CameraTrajectory.png
python3 $EVALUATE_SCRIPT --verbose $GROUDTRUTH_DATA_PATH $TRAJECTORY_PATH/KeyFrameTrajectory.txt --save_transformation transformation.txt --plot KeyFrameTrajectory.png

# for evo info https://github.com/MichaelGrupp/evo/wiki/evo_traj
#evo_ape tum $TRAJECTORY_PATH/CameraTrajectory.txt $GROUDTRUTH_DATA_PATH -a  # without s!

if [ $PLOT -eq 1 ] ; then
	evo_traj tum $TRAJECTORY_PATH/CameraTrajectory.txt --ref=$GROUDTRUTH_DATA_PATH -a -p --plot_mode=xz  &> /dev/null &
	evo_ape tum $TRAJECTORY_PATH/CameraTrajectory.txt $GROUDTRUTH_DATA_PATH -a -va --plot --plot_mode=xz &> /dev/null  &
fi	

# move results into output folder
if [ ! -d Results ]; then 
	mkdir Results 
fi 
mv CameraTrajectory* Results/
mv KeyFrameTrajectory* Results/
mv transformation.txt Results/
mv Performances.txt Results/
