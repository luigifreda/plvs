#!/bin/bash

# N.B.: 
# 1) you can kill all the spawned terminals together by right clicking on
# the X icon on the left bar and selecting "Quit"

#echo "usage: ./${0##*/} "

source ../ros_ws/devel/setup.bash

USE_SVO=1

#DATASET_BASE_FOLDER="$HOME/Work/datasets/rgbd_datasets/zed"
DATASET_BASE_FOLDER="~"
DATASET="zed"   # prefix 


# check if we want SVO output and give it a useful name 
if [ $USE_SVO -eq 1 ]; then 
    TIME_NOW=$(date +"%Y%m%d%H%M%S") 
    DATASET=$DATASET$TIME_NOW".svo"
fi  


DATASET_PATH=$DATASET_BASE_FOLDER/$DATASET
eval DATASET_PATH=$DATASET_PATH
echo recording file $DATASET_PATH


# check if zed wrapper has been launched otherwise launch it 
ZED_ROS_ON=$(rosnode list | grep zed_node)
if [ -z "$ZED_ROS_ON" ]; then 
    echo launching zed ros 
    xterm -e "echo LIVE ; roslaunch zed_wrapper zed.launch; bash" &
    sleep 5
fi 


if [ $USE_SVO -eq 0 ]; then  
    #xterm -e " echo 'RECORD'; cd $DATASET_BASE_FOLDER; rosbag record -o $DATASET /tf /zed/left/camera_info /zed/depth/camera_info /zed/left/image_rect_color /zed/depth/depth_registered ; bash" &
    xterm -e " echo 'RECORD'; cd $DATASET_BASE_FOLDER; rosbag record -o $DATASET /tf /zed/zed_node/rgb/camera_info /zed/zed_node/rgb/image_rect_color /zed/zed_node/depth/depth_registered; bash" &
else
    rosservice call /zed/zed_node/start_svo_recording $DATASET_PATH # Start recording an SVO file. If no filename is provided the default zed.svo is used. 
                                                                    # If no path is provided with the filename the default recording folder is ~/.ros/
fi 

rqt_image_view &

read -rsp $'Press any key to continue...\n' -n1 key

rosservice call /zed/zed_node/stop_svo_recording  # use this is order to stop the recording 

killall -9 rqt_image_view 
killall -9 xterm