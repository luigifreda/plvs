#!/usr/bin/env bash

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd ) # get script dir 
SCRIPT_DIR=$(readlink -f $SCRIPT_DIR)  # this reads the actual path if a symbolic directory is used
cd $SCRIPT_DIR # this brings us in the actual used folder (not the symbolic one)


EVALUATE_SCRIPT="$SCRIPT_DIR/evaluate_ate.py"
TRAJECTORY_DIR=$SCRIPT_DIR
DATASET_PATH=$1
TRAJECTORY_OUTPUT=$2
PLOT=$3


if [[ -n "$DATASET_PATH" ]]; then
	echo DATASET_PATH: $DATASET_PATH
else
	echo missing groundtruth path...
	exit 1
fi

if [[ -n "$PLOT" ]]
then
	echo "..."
else
    PLOT=0 # default 
fi


#GROUDTRUTH_DATA_PATH=$DATASET_PATH/mav0/state_groundtruth_estimate0/data.tum   # data generated with Scripts/generate_euroc_groundtruths_as_tum.sh
GROUDTRUTH_DATA_PATH=$DATASET_PATH/mav0/state_groundtruth_estimate0/data_t2.tum # data generated with Scripts/generate_euroc_groundtruths_as_tum.sh
F_CAMERA_TRAJECTORY=$TRAJECTORY_DIR/f_$TRAJECTORY_OUTPUT
KF_CAMERA_TRAJECTORY=$TRAJECTORY_DIR/kf_$TRAJECTORY_OUTPUT

#echo GROUDTRUTH_DATA_PATH: $GROUDTRUTH_DATA_PATH
#echo F_CAMERA_TRAJECTORY: $F_CAMERA_TRAJECTORY
#echo KF_CAMERA_TRAJECTORY: $KF_CAMERA_TRAJECTORY

python $EVALUATE_SCRIPT --verbose $GROUDTRUTH_DATA_PATH $F_CAMERA_TRAJECTORY --save_transformation transformation.txt --plot CameraTrajectory.png
python $EVALUATE_SCRIPT --verbose $GROUDTRUTH_DATA_PATH $KF_CAMERA_TRAJECTORY --save_transformation transformation.txt --plot KeyFrameTrajectory.png

# for evo info https://github.com/MichaelGrupp/evo/wiki/evo_traj
#evo_ape tum $TRAJECTORY_DIR/CameraTrajectory.txt $GROUDTRUTH_DATA_PATH -a # without s!

if [ $PLOT -eq 1 ] ; then
	evo_traj tum $F_CAMERA_TRAJECTORY  --ref=$GROUDTRUTH_DATA_PATH -a -p --plot_mode=xz  &> /dev/null &
	evo_ape tum $KF_CAMERA_TRAJECTORY $GROUDTRUTH_DATA_PATH -a -va --plot --plot_mode=xz &> /dev/null  &
fi	


# move results into output folder
if [ ! -d Results ]; then 
	mkdir Results 
fi 
mv CameraTrajectory* Results/
mv KeyFrameTrajectory* Results/
mv transformation.txt Results/
mv Performances.txt Results/