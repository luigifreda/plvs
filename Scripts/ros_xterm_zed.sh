#!/usr/bin/env bash

# N.B.:
# 1) you can kill all the spawned terminals together by right clicking on
# the X icon on the left bar and selecting "Quit"

#echo "usage: ./${0##*/} "

source ../ros_ws/devel/setup.bash

USE_LIVE=0

#DATASET_BASE_FOLDER="$HOME/Work/datasets/rgbd_datasets/zed"
#DATASET="flight_of_stairs.bag"

DATASET_BASE_FOLDER="$HOME/Work/datasets/rgbd_datasets/zed"
DATASET="2018-08-01-11-36-42.bag"

#DATASET_BASE_FOLDER="$HOME/Work/datasets/rgbd_datasets/zed"
#DATASET="zed20191228171358.svo"

DATASET_PATH=$DATASET_BASE_FOLDER/$DATASET
eval DATASET_PATH=$DATASET_PATH

#ROS_BAG_PLAY_OPTIONS="--rate 0.5 --pause"  # comment this to remove rate adjustment (pause on start)
#ROS_BAG_PLAY_OPTIONS="--rate 0.4 --start=240"  # comment this to remove rate adjustment (start from X secs)
#ROS_BAG_PLAY_OPTIONS="--rate 0.5"

export CAMERA_SETTINGS="../Settings/zed.yaml"
#export CAMERA_SETTINGS="../Settings/zed_vga.yaml"

export REMAP_COLOR_TOPIC="/camera/rgb/image_raw:=/zed/zed_node/rgb/image_rect_color"
export REMAP_DEPTH_TOPIC="/camera/depth_registered/image_raw:=/zed/zed_node/depth/depth_registered"
export REMAP_CAMERA_INFO_TOPIC="/camera/rgb/camera_info:=/zed/zed_node/rgb/camera_info"

# check if we are using SVO files 
USE_SVO_FILE=0
if [[ $DATASET_PATH == *".svo" ]]; then
    USE_SVO_FILE=1
fi

# manage topic remapping with old zed bags  
if [ $USE_LIVE -eq 0 ] && [ $USE_SVO_FILE -eq 0 ]; then  
	USE_OLD_TOPIC_NAMES=$(rosbag info $DATASET_PATH | grep /zed/left/image_rect_color)  # check if bag uses old topic naming
	#echo USE_OLD_TOPIC_NAMES: $USE_OLD_TOPIC_NAMES
	if [ ! -z "$USE_OLD_TOPIC_NAMES" ]; then 
		echo 'using old zed camera topics'
		REMAP_COLOR_TOPIC="/camera/rgb/image_raw:=/zed/left/image_rect_color"
		REMAP_DEPTH_TOPIC="/camera/depth_registered/image_raw:=/zed/depth/depth_registered"
		REMAP_CAMERA_INFO_TOPIC="/camera/rgb/camera_info:=/zed/left/camera_info"    
	fi
fi

# ======================================================================
# for debugging

#export DEBUG_PREFIX="--prefix 'gdb -ex run --args'"  # uncomment this in order to debug with gdb
#export LOG_SUFFIX="&> plvs_log.txt"

# ======================================================================

xterm -e "echo ROSCORE ; roscore ; bash" &
sleep 3

if [ $USE_LIVE -eq 0 ]; then
    # set before launching any node    (https://answers.ros.org/question/217588/error-in-rosbag-play-despite-setting-use_sim_time-param/)
    #rosparam set use_sim_time true
    sleep 1
fi

# ======================================================================

xterm -e "echo plvs ; rosrun $DEBUG_PREFIX  plvs RGBD ../Vocabulary/ORBvoc.txt $CAMERA_SETTINGS  $REMAP_COLOR_TOPIC  $REMAP_DEPTH_TOPIC  $REMAP_CAMERA_INFO_TOPIC $LOG_SUFFIX; bash" &

# ======================================================================

if [ $USE_LIVE -eq 1 ]; then
    xterm -e "echo LIVE ; roslaunch zed_wrapper zed.launch; bash" &
else
    if [ $USE_SVO_FILE -eq 1 ]; then
        echo using svo file $DATASET_PATH 
        xterm -e "echo RECORDED svo; roslaunch zed_wrapper zed.launch svo_file:=$DATASET_PATH ; bash" &
    else
        echo using bag file
        rosparam set use_sim_time true
        xterm -e "echo RECORDED bag; rosbag play --clock -d 1 $DATASET_PATH $ROS_BAG_PLAY_OPTIONS; bash" &
    fi
fi

echo "DONE "

# NOTE: you can use the following command to get the xterm window live if the app terminates or crashes
# xterm -e "<you_command>; bash" &


# record "/zed/depth/depth_registered","/zed/left/image_rect_color"
