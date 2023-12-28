#!/usr/bin/env bash

# N.B.:
# 1) you can kill all the spawned terminals together by right clicking on
# the X icon on the left bar and selecting "Quit"

#echo "usage: ./${0##*/} "

source ../ros_ws/devel/setup.bash

USE_LIVE=1
USE_RVIZ=0   # if you set this to 1, you should also set Viewer.on: 0 in the yaml settings


DATASET_BASE_FOLDER="$HOME/Work/datasets/rgbd_datasets/dataset_oak/"
#ROS_BAG_PLAY_OPTIONS="--rate 0.5"  # comment this to remove rate adjustment
DATASET=""

DATASET_PATH=$DATASET_BASE_FOLDER/$DATASET
eval DATASET_PATH=$DATASET_PATH

#export CAMERA_SETTINGS="../Settings/oak-d-vga.yaml"  # NOTE: it does not work at the moment. Depth does not come aligned with RGB image. 
export CAMERA_SETTINGS="../Settings/oak-d.yaml"

export REMAP_COLOR_TOPIC="/camera/rgb/image_raw:=/oak/rgb/image_rect"
#export REMAP_COLOR_TOPIC="/camera/rgb/image_raw:=/oak/rgb/image_raw"
export REMAP_DEPTH_TOPIC="camera/depth_registered/image_raw:=/oak/stereo/image_raw"    # we use rectify_rgb:=true below
export REMAP_CAMERA_INFO="/camera/rgb/camera_info:=/oak/stereo/camera_info"

#export DEBUG_PREFIX="--prefix 'gdb -ex run --args'"  # uncomment this in order to debug with gdb

# ======================================================================

xterm -e "echo ROSCORE ; roscore ; bash" &
sleep 3

if [ $USE_LIVE -eq 0 ]; then
	# set before launching any node    (https://answers.ros.org/question/217588/error-in-rosbag-play-despite-setting-use_sim_time-param/)
    #rosparam set use_sim_time true
    sleep 1
fi

# ======================================================================

if [ $USE_LIVE -eq 1 ]; then
    #xterm -e "echo LIVE; roslaunch depthai_ros_driver rgbd_pcl.launch rectify_rgb:=true ; bash" &    # started from this 
    xterm -e "echo LIVE; roslaunch plvs oak-d.launch; bash" &     # we use rectify_rgb:=true inside
    #sleep 5
else
    sleep 5
    rosparam set use_sim_time true
    xterm -e "echo RECORDED; rosbag play --clock $ROS_BAG_PLAY_OPTIONS $DATASET_PATH; bash" &
fi

# NOTE: you can use the following command to get the xterm window live if the app terminates or crashes
# xterm -e "<you_command>; bash" &

# ======================================================================

# wait a bit the OAK ros driver to wake up 
xterm -e "echo plvs ; rosrun $DEBUG_PREFIX  plvs RGBD ../Vocabulary/ORBvoc.bin $CAMERA_SETTINGS $REMAP_CAMERA_INFO $REMAP_COLOR_TOPIC $REMAP_DEPTH_TOPIC; bash" &

# ======================================================================

if [ $USE_RVIZ -eq 1 ]; then
    xterm -e "echo RVIZ ; roslaunch plvs rviz_plvs.launch ; bash" &
fi

# ======================================================================

echo "DONE "



