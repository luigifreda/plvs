#!/usr/bin/env bash

# N.B.: 
# 1) you can kill all the spawned terminals together by right clicking on
# the X icon on the left bar and selecting "Quit"

#echo "usage: ./${0##*/} "

source ../ros_ws/devel/setup.bash


DATASET_BASE_FOLDER="$HOME/Work/datasets/rgbd_datasets/tum"


# IMPORTANT: Refer to https://github.com/ros/ros_comm/issues/82
# Playing TUM rosbags at normal rate is not possible since the clock publication is not reliable. 
# In order to solve the problem the bags must be decompressed with rosbag decompress! 
# A quick fix is to play the bags with half rate (it's very likely this gives time to the decoding process to finish its job on time so that clock data publication can be be normally performed.


DATASET="rgbd_dataset_freiburg1_room" 
#DATASET='rgbd_dataset_freiburg2_pioneer_slam'
#DATASET='rgbd_dataset_freiburg2_pioneer_slam3'
#DATASET='rgbd_dataset_freiburg3_structure_notexture_far'

RGBD_DATASET_PATH=$DATASET_BASE_FOLDER/$DATASET/$DATASET".bag"

ROS_BAG_PLAY_OPTIONS="--rate 1"  # comment this to remove rate adjustment 

export CAMERA_SETTINGS="../Settings/ros/TUM1_ros.yaml"

if [[ $DATASET == *freiburg2* ]]; then
	echo "using TUM2"
	CAMERA_SETTINGS="../Settings/ros/TUM2_ros.yaml"
fi
if [[ $DATASET == *freiburg3* ]]; then
	echo "using TUM3"
	CAMERA_SETTINGS="../Settings/ros/TUM3_ros.yaml"
fi

echo "CAMERA_SETTINGS: $CAMERA_SETTINGS"

export REMAP_COLOR_TOPIC="/camera/rgb/image_raw:=/camera/rgb/image_color "
export REMAP_DEPTH_TOPIC="camera/depth_registered/image_raw:=/camera/depth/image "

#export DEBUG_PREFIX="--prefix 'gdb -ex run --args'"  # uncomment this in order to debug with gdb

# ======================================================================

if [ ! -f $RGBD_DATASET_PATH ]; then 
	echo bag $RGBD_DATASET_PATH does not exist!
	continue 
fi 

echo processing bag $RGBD_DATASET_PATH

xterm -T "roscore" -e "echo ROSCORE ; roscore ; bash" &
./wait_for_rosmaster.sh

# set before launching any node    (https://answers.ros.org/question/217588/error-in-rosbag-play-despite-setting-use_sim_time-param/)
#rosparam set use_sim_time true

# ======================================================================

xterm -T "plvs RGBD" -e "echo plvs ; rosrun $DEBUG_PREFIX  plvs RGBD ../Vocabulary/ORBvoc.txt $CAMERA_SETTINGS  $REMAP_COLOR_TOPIC  $REMAP_DEPTH_TOPIC; bash" &
#xterm -e "echo plvs ;rqt_image_view; bash" &  # just for checking

# ======================================================================

sleep 5
rosparam set use_sim_time true 
xterm -T "rosbag play" -e "echo RECORDED ; rosbag play --clock -d 1 $ROS_BAG_PLAY_OPTIONS $RGBD_DATASET_PATH; bash" & 
#xterm -T "rosbag play" -e "echo RECORDED ; rosbag play -d 1 $ROS_BAG_PLAY_OPTIONS $RGBD_DATASET_PATH; bash" & 

# NOTE: you can use the following command to get the xterm window live if the app terminates or crashes
# xterm -e "<you_command>; bash" &


# ======================================================================

echo "DONE "



