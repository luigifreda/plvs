#!/usr/bin/env bash

# N.B.:
# 1) you can kill all the spawned terminals together by right clicking on
# the X icon on the left bar and selecting "Quit"

#echo "usage: ./${0##*/} "

#source ../ros_ws/devel/setup.bash  # this makes openni crash since we use a custom version of opencv and vision_opencv (w.r.t. openni and openni2)

USE_LIVE=1
USE_RVIZ=0   # if you set this to 1, you should also set Viewer.on: 0 in the yaml settings


# possible dataset smallOfficeDIAG.bag desktop-change.bag
DATASET_BASE_FOLDER="$HOME/Work/datasets/rgbd_datasets/xtion/"
#ROS_BAG_PLAY_OPTIONS="--rate 0.5"  # comment this to remove rate adjustment
DATASET="danger_zone.bag"
#DATASET="desktop-change.bag"

DATASET_PATH=$DATASET_BASE_FOLDER/$DATASET
eval DATASET_PATH=$DATASET_PATH

export CAMERA_SETTINGS="../Settings/xtion.yaml"

export REMAP_COLOR_TOPIC="/camera/rgb/image_raw:=/camera/rgb/image_rect_color"
export REMAP_DEPTH_TOPIC="camera/depth_registered/image_raw:=/camera/depth_registered/sw_registered/image_rect"

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

xterm -e "echo plvs; source ../ros_ws/devel/setup.bash; rosrun $DEBUG_PREFIX  plvs RGBD ../Vocabulary/ORBvoc.bin $CAMERA_SETTINGS  $REMAP_COLOR_TOPIC  $REMAP_DEPTH_TOPIC; bash" &

# ======================================================================

if [ $USE_LIVE -eq 1 ]; then
    xterm -e "echo LIVE; roslaunch openni2_launch openni2.launch ; bash" &  
    #xterm -e "echo LIVE; roslaunch openni_launch openni.launch ; bash" &  # !this is much slower, use openni2!
else
    sleep 5
    rosparam set use_sim_time true
    xterm -e "echo RECORDED; rosbag play --clock $ROS_BAG_PLAY_OPTIONS $DATASET_PATH; bash" &
fi

# NOTE: you can use the following command to get the xterm window live if the app terminates or crashes
# xterm -e "<you_command>; bash" &

# ======================================================================

if [ $USE_RVIZ -eq 1 ]
then
    xterm -e "echo RVIZ; source ../ros_ws/devel/setup.bash; roslaunch plvs rviz_plvs.launch ; bash" &
fi

# ======================================================================

echo "DONE "

# record "/camera/rgb/image_rect_color",/camera/depth_registered/sw_registered/image_rect"

