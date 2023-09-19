#!/bin/bash

# N.B.: 
# 1) you can kill all the spawned terminals together by right clicking on
# the X icon on the left bar and selecting "Quit"

#echo "usage: ./${0##*/} "

source ../ros_ws/devel/setup.bash

# possible datasets: 
DATASET='rgbd_dataset_freiburg2_pioneer_slam'
RGBD_DATASET_PATH="$HOME/Work/datasets/rgbd_datasets/$DATASET/$DATASET.bag"

#ROS_BAG_PLAY_OPTIONS="--rate 0.5"  # comment this to remove rate adjustment 

export CAMERA_SETTINGS="../Settings/ros/TUM2_ros.yaml"

export REMAP_COLOR_TOPIC="/camera/image_raw:=/camera/rgb/image_color "
#export REMAP_DEPTH_TOPIC="camera/depth_registered/image_raw:=/camera/depth/image "

#export DEBUG_PREFIX="--prefix 'gdb -ex run --args'"  # uncomment this in order to debug with gdb

# ======================================================================

xterm -e "echo ROSCORE ; roscore ; bash" &
sleep 3

# ======================================================================


xterm -e "echo plvs ; rosrun $DEBUG_PREFIX  plvs MonoAR ../Vocabulary/ORBvoc.txt $CAMERA_SETTINGS  $REMAP_COLOR_TOPIC  $REMAP_DEPTH_TOPIC; bash" &

# ======================================================================

sleep 8
rosparam set use_sim_time true 
xterm -e "echo RECORDED ; rosbag play --clock $ROS_BAG_PLAY_OPTIONS $RGBD_DATASET_PATH; bash" & 


# NOTE: you can use the following command to get the xterm window live if the app terminates or crashes
# xterm -e "<you_command>; bash" &


# ======================================================================

echo "DONE "



