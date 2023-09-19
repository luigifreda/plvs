#!/usr/bin/env bash

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd ) # get script dir 
SCRIPT_DIR=$(readlink -f $SCRIPT_DIR)  # this reads the actual path if a symbolic directory is used
cd $SCRIPT_DIR # this brings us in the actual used folder (not the symbolic one)


TRAJECTORY_PATH=$SCRIPT_DIR
GROUDTRUTH_PATH=$1
IS_MONO=$2
PLOT=$3

if [[ -n "$GROUDTRUTH_PATH" ]]
then
	echo "..."
else
	echo missing groundtruth path...
	exit 1
fi

if [[ -n "$IS_MONO" ]]
then
	echo "using mono options"
else
    IS_MONO=0 # default 
fi

if [[ -n "$PLOT" ]]
then
	echo "..."
else
    PLOT=0 # default 
fi

#echo TRAJECTORY_PATH=$TRAJECTORY_PATH
echo GROUDTRUTH_PATH=$GROUDTRUTH_PATH


# for evo info https://github.com/MichaelGrupp/evo/wiki/evo_traj

if [ $PLOT -eq 1 ] ; then
	evo_traj kitti $TRAJECTORY_PATH/CameraTrajectory.txt --ref=$GROUDTRUTH_PATH -a  -p --plot_mode=xz &> /dev/null  &
	evo_ape kitti $TRAJECTORY_PATH/CameraTrajectory.txt $GROUDTRUTH_PATH -a  -va --plot --plot_mode=xz &> /dev/null  &
fi	

if [ $IS_MONO -eq 1 ] ; then
	# use the script generate_kitti_groundtruths_as_tum.sh to generate the groundtruth files in tum format
	GROUDTRUTH_PATH="$(dirname $GROUDTRUTH_PATH)/gt_tum.txt"
	evo_ape tum $TRAJECTORY_PATH/KeyFrameTrajectory.txt $GROUDTRUTH_PATH -as # -s to align in Sim(3) given the monocular sensor
else 
	evo_ape kitti $TRAJECTORY_PATH/CameraTrajectory.txt $GROUDTRUTH_PATH -a 
fi 


# move results into output folder
if [ ! -d Results ]; then 
	mkdir Results 
fi 
mv CameraTrajectory* Results/
mv KeyFrameTrajectory* Results/
mv transformation.txt Results/
mv Performances.txt Results/