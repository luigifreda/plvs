#!/bin/bash

# N.B.: 
# 1) you can kill all the spawned terminals together by right clicking on
# the X icon on the left bar and selecting "Quit"

#echo "usage: ./${0##*/} "

RGBD_DATASET_FOLDER="$HOME/Work/datasets/rgbd_datasets/DIAG_xtion"

DATASET="new"

xterm -e " echo "RECORD"; cd $RGBD_DATASET_FOLDER; rosbag record -o $DATASET /tf /camera/rgb/camera_info /camera/depth/camera_info /camera/depth_registered/camera_info /camera/depth_registered/sw_registered/camera_info /camera/depth_registered/sw_registered/image_rect /camera/rgb/image_rect_color ; bash" &
