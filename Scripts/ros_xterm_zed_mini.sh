#!/bin/bash

# N.B.:
# 1) you can kill all the spawned terminals together by right clicking on
# the X icon on the left bar and selecting "Quit"

#echo "usage: ./${0##*/} "

source ../ros_ws/devel/setup.bash

USE_LIVE=0

RGBD_DATASET_FOLDER="~/Work/datasets/rgbd_datasets/rgbd-mini"

DATASET="new_2018-09-27-11-40-10.bag"  


#ROS_BAG_PLAY_OPTIONS="--rate 0.5"  # comment this to remove rate adjustment

export CAMERA_SETTINGS="../Settings/zed_mini.yaml"
#export CAMERA_SETTINGS="../Settings/zed_vga.yaml"

export REMAP_COLOR_TOPIC="/camera/rgb/image_raw:=/zed/left/image_rect_color"
export REMAP_DEPTH_TOPIC="/camera/depth_registered/image_raw:=/zed/depth/depth_registered"
export REMAP_CAMERA_INFO_TOPIC="/camera/rgb/camera_info:=/zed/left/camera_info"

#export DEBUG_PREFIX="--prefix 'gdb -ex run --args'"  # uncomment this in order to debug with gdb
#export LOG_SUFFIX="&> plvs_log.txt"

# ======================================================================

xterm -e "echo ROSCORE ; roscore ; bash" &
sleep 3

if [ $USE_LIVE -eq 0 ]
then
	# set before launching any node    (https://answers.ros.org/question/217588/error-in-rosbag-play-despite-setting-use_sim_time-param/)
    #rosparam set use_sim_time true
    sleep 1
fi

# ======================================================================

xterm -e "echo plvs ; rosrun $DEBUG_PREFIX  plvs RGBD ../Vocabulary/ORBvoc.txt $CAMERA_SETTINGS  $REMAP_COLOR_TOPIC  $REMAP_DEPTH_TOPIC  $REMAP_CAMERA_INFO_TOPIC $LOG_SUFFIX; bash" &

# ======================================================================

if [ $USE_LIVE -eq 1 ]
then
    xterm -e "echo LIVE ; roslaunch zed_wrapper zed.launch camera_model:=1 camera_disable_self_calib:=false; bash" &
else
    sleep 1
    rosparam set use_sim_time true
    xterm -e "echo RECORDED ; rosbag play --clock $RGBD_DATASET_FOLDER/$DATASET $ROS_BAG_PLAY_OPTIONS; bash" &
fi

echo "DONE "

# NOTE: you can use the following command to get the xterm window live if the app terminates or crashes
# xterm -e "<you_command>; bash" &


# record "/zed/depth/depth_registered","/zed/left/image_rect_color"
