#!/bin/bash

# N.B.:
# 1) you can kill all the spawned terminals together by right clicking on
# the X icon on the left bar and selecting "Quit"

#echo "usage: ./${0##*/} "

source ../ros_ws/devel/setup.bash

RGBD_DATASET_FOLDER="$HOME/datasets/rgbd_datasets/zed/rgbd"

#DATASET="new_2018-08-01-10-43-12.bag" # palina ingresso
#DATASET="new_2018-08-01-11-32-37.bag" # tribune sotto
#DATASET="new_2018-08-01-11-36-42.bag" # tabernae
#DATASET="new_2018-08-01-11-45-46.bag" # sopra e sotto

#DATASET="new_2018-09-04-12-40-15.bag"  # part 1  arco di tito, torre e tribune
#DATASET="new_2018-09-04-13-05-40.bag"  # part 2 arco di tito, torre e tribune

#DATASET="new_2018-09-13-10-12-28.bag" # arco di tito dettaglio 1 - sezione 7
#DATASET="new_2018-09-13-11-38-37.bag"  # arco di tito dettaglio 2 - sezione 7
#DATASET="new_2018-09-13-10-34-55.bag" # tribune dettaglio
#DATASET="new_2018-09-13-10-52-01.bag" # sezione 4

DATASET="new_2018-09-27-10-45-36.bag"  # sezione 4
#DATASET="new_2018-09-27-10-59-08.bag" # tribune
#DATASET="new_2018-09-27-11-11-43.bag" # tabernae
#DATASET="new_2018-09-27-12-21-52.bag" # sezione 4 solo attorno palina (strano comportamento?)
#DATASET="new_2018-09-27-12-42-32.bag" # sezione 4 solo attorno palina

#DATASET="new_2018-10-12-11-32-11.bag" # da s7 a schiena centrale (track lost vicino a recinto)
#DATASET="new_2018-10-12-12-02-26.bag" # da sotto s7 verso ala sinistra
#DATASET="new_2018-10-12-12-23-06.bag" # s7 intorno al piano della torre
#DATASET="new_2018-10-12-12-37-48.bag" # da sotto s7 verso ala destra


ROS_BAG_PLAY_OPTIONS="--rate 0.5"  # comment this to remove rate adjustment


export REMAP_COLOR_TOPIC="image_topic:=/zed/left/image_rect_color"
export REMAP_DEPTH_TOPIC="depth_topic:=/zed/depth/depth_registered"

# ======================================================================

xterm -e "echo ROSCORE ; roscore ; bash" &
sleep 1

# ======================================================================

#xterm -e "echo plvs ; rosrun $DEBUG_PREFIX  plvs RGBD ../Vocabulary/ORBvoc.txt $CAMERA_SETTINGS  $REMAP_COLOR_TOPIC  $REMAP_DEPTH_TOPIC; bash" &

# ======================================================================

rosparam set use_sim_time true
xterm -e "echo RECORDED ; rosbag play --clock $RGBD_DATASET_FOLDER/$DATASET $ROS_BAG_PLAY_OPTIONS; bash" &

# ======================================================================

xterm -e "echo save images ; roslaunch plvs save_images_from_bag.launch bag:=$RGBD_DATASET_FOLDER/$DATASET $REMAP_COLOR_TOPIC; bash" &

# ======================================================================

echo "DONE "

# NOTE: you can use the following command to get the xterm window live if the app terminates or crashes
# xterm -e "<you_command>; bash" &


# record "/zed/depth/depth_registered","/zed/left/image_rect_color"
