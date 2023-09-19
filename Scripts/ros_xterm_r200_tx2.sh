#!/bin/bash

# N.B.: 
# 1) you can kill all the spawned terminals together by right clicking on
# the X icon on the left bar and selecting "Quit"

#echo "usage: ./${0##*/} "

# possible dataset smallOfficeDIAG.bag bigOfficeDIAG.bag outdoorDIAG.bag

USE_LIVE=0
RGBD_DATASET_FOLDER="$HOME/Work/datasets/rgbd_datasets"

# ======================================================================

xterm -e "echo ROSCORE ; roscore ; bash" &
sleep 2

# ======================================================================

xterm -e "echo plvs ; source ../ros_ws/devel/setup.bash; rosrun plvs R200 ../Vocabulary/ORBvoc.txt ../Settings/r200.yaml ; bash" &

# ======================================================================

if [ $USE_LIVE -eq 1 ] 
then
    xterm -e "echo LIVE ; source ../../../tx1_wss/realsense_ws_tx1/devel/setup.bash ; roslaunch realsense_camera r200_nodelet_rgbd.launch ; bash" & 
else 
	sleep 8
    xterm -e "echo RECORDED ; rosbag play $RGBD_DATASET_FOLDER/DIAG_r200/bigOfficeDIAG.bag ; bash" & 
fi 

echo "DONE "

# NOTE: you can use the following command to get the xterm window live if the app terminates or crashes
# xterm -e "<you_command>; bash" &

