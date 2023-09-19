#!/bin/bash

# N.B.: 
# 1) you can kill all the spawned terminals together by right clicking on
# the X icon on the left bar and selecting "Quit"

#echo "usage: ./${0##*/} "

#RGBD_DATASET_FOLDER="$HOME/Work/datasets/rgbd_datasets/d435"
RGBD_DATASET_FOLDER="~"

DATASET="new"

xterm -e " echo "RECORD"; cd $RGBD_DATASET_FOLDER; rosbag record -o $DATASET /tf  /camera/aligned_depth_to_color/camera_info \
																				/camera/color/camera_info \
																				/camera/depth/camera_info \
                                                                                /camera/aligned_depth_to_color/image_raw \
                                                                                /camera/color/image_rect_color ; bash" &

