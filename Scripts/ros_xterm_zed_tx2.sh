#!/bin/bash

# N.B.: 
# 1) you can kill all the spawned terminals together by right clicking on
# the X icon on the left bar and selecting "Quit"

#echo "usage: ./${0##*/} "

# possible dataset inOutDIAG.bag outdoorDIAG.bag

USE_LIVE=0
RGBD_DATASET_FOLDER="$HOME/Work/datasets/rgbd_datasets"

# ======================================================================

xterm -e "echo ROSCORE ; roscore ; bash" &
sleep 2

# ======================================================================

xterm -e "echo plvs ; source ../ros_ws/devel/setup.bash ; rosrun plvs ZED ../Vocabulary/ORBvoc.txt ../Settings/zed.yaml ; bash" &

# ======================================================================

if [ $USE_LIVE -eq 1 ] 
then
    xterm -e "echo LIVE ; source ../../../zed_wss/devel/setup.bash; roslaunch zed_wrapper zed.launch ; bash" & 
else 
    sleep 30
    rosparam set use_sim_time true  	 
    xterm -e "echo RECORDED ; rosbag play --clock $RGBD_DATASET_FOLDER/DIAG_zed/inOutDIAG.bag $ROB_BAG_PLAY_OPTIONS; bash" & 
    #xterm -e "echo RECORDED ; rosbag play --clock $RGBD_DATASET_FOLDER/DIAG_zed/outDIAG.bag $ROB_BAG_PLAY_OPTIONS; bash" &
fi 

echo "DONE "

# NOTE: you can use the following command to get the xterm window live if the app terminates or crashes
# xterm -e "<you_command>; bash" &


