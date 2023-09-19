#!/bin/bash

# N.B.:
# 1) you can kill all the spawned terminals together by right clicking on
# the X icon on the left bar and selecting "Quit"

#echo "usage: ./${0##*/} "

source ../ros_ws/devel/setup.bash

USE_LIVE=0
USE_ELAS_ROS=0

# possible dataset inOutDIAG.bag outDIAG.bag stairs_zed.bag small_office_zed.bag outdoor_vga

RGBD_DATASET_FOLDER="~/Work/datasets/zed"
DATASET="test.bag"

#ROS_BAG_PLAY_OPTIONS="--rate 0.5"  # comment this to remove rate adjustment

export CAMERA_SETTINGS="../Settings/zed2i.yaml"
#export CAMERA_SETTINGS="../Settings/zed_vga.yaml"

export REMAP_COLOR_TOPIC="/camera/left/image_raw:=/zed2i/zed_node/left/image_rect_color /camera/right/image_raw:=/zed2i/zed_node/right/image_rect_color"
#export REMAP_DEPTH_TOPIC=""

export STEREO_REMAP_TOPIC="stereo:=zed image:=image_rect_color"


#export DEBUG_PREFIX="--prefix 'gdb -ex run --args'"  # uncomment this in order to debug with gdb
#export DEBUG_PREFIX="valkyrie "


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

xterm -e "echo plvs ; rosrun $DEBUG_PREFIX  plvs Stereo ../Vocabulary/ORBvoc.txt $CAMERA_SETTINGS false $REMAP_COLOR_TOPIC  $REMAP_DEPTH_TOPIC; bash" &

# ======================================================================

if [ $USE_LIVE -eq 1 ]
then
    xterm -e "echo LIVE ; source ../../../zed_wss/devel/setup.bash; roslaunch zed_wrapper zed.launch ; bash" &
else
    sleep 8
    rosparam set use_sim_time true
    xterm -e "echo RECORDED ; rosbag play --clock $RGBD_DATASET_FOLDER/$DATASET $ROS_BAG_PLAY_OPTIONS; bash" &
fi

if [ $USE_ELAS_ROS -eq 1 ]
then
	xterm -e "echo elas ; roslaunch elas_ros elas.launch $STEREO_REMAP_TOPIC $STEREO_CALIB_REMAP_TOPIC; bash" &
	xterm -e "echo rviz ; roslaunch elas_ros rviz_zed.launch; bash" &	
fi	

echo "DONE "

# NOTE: you can use the following command to get the xterm window live if the app terminates or crashes
# xterm -e "<you_command>; bash" &


# record "/zed/depth/depth_registered","/zed/left/image_rect_color"
