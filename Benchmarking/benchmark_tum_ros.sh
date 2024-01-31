#!/bin/bash

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd ) # get script dir (this should be the main folder directory of PLVS)
SCRIPT_DIR=$(readlink -f $SCRIPT_DIR)  # this reads the actual path if a symbolic directory is used
cd $SCRIPT_DIR # this brings us in the actual used folder (not the possibly symbolic one)

NUM_RUNS_X_DATASET=3

DATASET_LIST_PATH=$1
if [[ -n "$DATASET_LIST_PATH" ]]; then
	echo "..."
else
	DATASET_LIST_PATH="dataset_list_single_test.txt"
fi

MAIN_DIR=$SCRIPT_DIR/..
EVALUATE_SCRIPT="$MAIN_DIR/Scripts/evaluate_ate.py"
KILLER_SCRIPT="$SCRIPT_DIR/killer.sh"

DATE=`date '+%Y-%m-%d-%H-%M'`
OUTPUT_FOLDER=results_ros_$DATE


if [ -d results ]; then 
    mv results results_unknown_$DATE 
fi 
mkdir -p results


# ROS start and setup 
source $MAIN_DIR/ros_ws/devel/setup.bash
xterm -T "roscore" -e "echo ROSCORE ; roscore ; bash" &
$MAIN_DIR/Scripts/wait_for_rosmaster.sh
export REMAP_COLOR_TOPIC="/camera/rgb/image_raw:=/camera/rgb/image_color "
export REMAP_DEPTH_TOPIC="camera/depth_registered/image_raw:=/camera/depth/image "


while IFS=$'\n' read -r line || [[ -n "$line" ]]; do
	echo $'\n=============================================================\n'
    echo "Dataset path: $line"
    RGBD_DATASET_PATH=$line
    eval RGBD_DATASET_PATH=$RGBD_DATASET_PATH
    
    if [[ $RGBD_DATASET_PATH == *"freiburg1"* ]]; then
        TUM_YAML="TUM1_ros.yaml"
    fi
    if [[ $RGBD_DATASET_PATH == *"freiburg2"* ]]; then
        TUM_YAML="TUM2_ros.yaml"
    fi
    if [[ $RGBD_DATASET_PATH == *"freiburg3"* ]]; then
        TUM_YAML="TUM3_ros.yaml"
    fi
    echo Used settings: $TUM_YAML

	DATASET=$(basename $RGBD_DATASET_PATH)
	echo Dataset name: $DATASET
    mkdir -p $DATASET
    cd $DATASET

    if [ ! -f "$RGBD_DATASET_PATH/$DATASET.bag" ]; then 
        echo bag $RGBD_DATASET_PATH/$DATASET.bag does not exist!
        continue 
    fi 

    for i in `seq 1 $NUM_RUNS_X_DATASET`; do
		echo $'\n..........................................................\n'
        echo "Starting run $i"
        OUTPUT_LOG="log$i.txt" 
        echo Logging to $OUTPUT_LOG

		xterm -T "rosbag play" -e "rosbag play --clock $RGBD_DATASET_PATH/$DATASET.bag; exit" &
		xterm -T "killer" -e "bash $KILLER_SCRIPT; exit" &
        xterm -T "system stats logger" -e "$MAIN_DIR/Scripts/system_stats_logger.py -p RGBD -o Resources.txt --sleep-start 5; exit" &
		xterm -T "RGBD" -e "rosrun $DEBUG_PREFIX plvs RGBD \
            $MAIN_DIR/Vocabulary/ORBvoc.txt \
            $MAIN_DIR/Settings/ros/$TUM_YAML \
            $REMAP_COLOR_TOPIC $REMAP_DEPTH_TOPIC 2>&1 >> $OUTPUT_LOG; exit"

		echo "Done with run $i"; sleep 1

		python $EVALUATE_SCRIPT --verbose $RGBD_DATASET_PATH/groundtruth.txt $PWD/CameraTrajectory.txt > "Ate.txt"

        mv Ate.txt $(printf "Ate_%d.txt" $i)
		mv Resources.txt $(printf "Resources_%d.txt" $i)
        mv CameraTrajectory.txt $(printf "CameraTrajectory_%d.txt" $i)
        mv KeyFrameTrajectory.txt $(printf "KeyFrameTrajectory_%d.txt" $i)

        sleep 2
    done

    # copy the settings 
    cp $MAIN_DIR/Settings/ros/$TUM_YAML . 

    cd ../
	mv $DATASET results/$DATASET

done < $DATASET_LIST_PATH

python generate_results_ros.py 2>&1 | tee ./results/Stats.txt

echo moving results to ouput folder $OUTPUT_FOLDER
mv results $OUTPUT_FOLDER

xmessage -fn -*-fixed-*-*-*-*-40-*-*-*-*-*-iso8859-* 'tum RGBD benchmarking completed'
