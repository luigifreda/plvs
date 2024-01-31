#!/bin/bash

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd ) # get script dir (this should be the main folder directory of PLVS)
SCRIPT_DIR=$(readlink -f $SCRIPT_DIR)  # this reads the actual path if a symbolic directory is used
cd $SCRIPT_DIR # this brings us in the actual used folder (not the possibly symbolic one)

SUFFIX="_old" # comment if you want to use the new example binaries

NUM_RUNS_X_DATASET=10

DATASET_LIST_PATH=$1
if [[ -n "$DATASET_LIST_PATH" ]]; then
	echo "..."
else
	DATASET_LIST_PATH="dataset_list_single_test.txt"
fi

MAIN_DIR=$SCRIPT_DIR/..
EVALUATE_SCRIPT="$MAIN_DIR/Scripts/evaluate_ate.py"

DATE=`date '+%Y-%m-%d-%H-%M'`
OUTPUT_FOLDER=results_$DATE

if [ -d results ]; then 
    mv results results_unknown_$DATE 
fi 
mkdir -p results


while IFS=$'\n' read -r line || [[ -n "$line" ]]; do
	echo $'\n=============================================================\n'
    echo "Dataset path: $line"
    RGBD_DATASET_PATH=$line
    eval RGBD_DATASET_PATH=$RGBD_DATASET_PATH

    if [[ $RGBD_DATASET_PATH == *"freiburg1"* ]]; then
        TUM_YAML="TUM1.yaml"
    fi
    if [[ $RGBD_DATASET_PATH == *"freiburg2"* ]]; then
        TUM_YAML="TUM2.yaml"
    fi
    if [[ $RGBD_DATASET_PATH == *"freiburg3"* ]]; then
        TUM_YAML="TUM3.yaml"
    fi
    echo Used settings: $TUM_YAML

    DATASET=$(basename $RGBD_DATASET_PATH)
    echo Dataset name: $DATASET
    mkdir -p $DATASET
    cd $DATASET

    echo Number of runs x dataset: $NUM_RUNS_X_DATASET

    for i in `seq 1 $NUM_RUNS_X_DATASET`; do
	    echo $'\n..........................................................\n'
        echo "Starting run $i"
        OUTPUT_LOG="log$i.txt" 
        OUTPUT_SYS_LOG="log_sys_stats$i.txt"
        echo Logging plvs to $OUTPUT_LOG
        echo Logging system stats to $OUTPUT_SYS_LOG        

        $MAIN_DIR/Scripts/system_stats_logger.py -p rgbd_tum$SUFFIX -o Resources.txt --sleep-start 1  2>&1 >> $OUTPUT_SYS_LOG &

        # launch rgbd_tum 
        $MAIN_DIR/Examples$SUFFIX/RGB-D/rgbd_tum$SUFFIX \
            $MAIN_DIR/Vocabulary/ORBvoc.txt \
            $MAIN_DIR/Examples$SUFFIX/RGB-D/$TUM_YAML \
            $RGBD_DATASET_PATH $RGBD_DATASET_PATH/associations.txt 2>&1 >> $OUTPUT_LOG && fg
        
        sleep 1; echo -e "\n"  # this is used to pass a char input and close the process (not needed without viewer)
        
		echo "Done with run $i"; sleep 1

        python $EVALUATE_SCRIPT --verbose $RGBD_DATASET_PATH/groundtruth.txt $PWD/CameraTrajectory.txt > "Ate.txt"

        mv Ate.txt $(printf "Ate_%d.txt" $i)
        mv Resources.txt $(printf "Resources_%d.txt" $i)
        mv Performances.txt $(printf "Performances_%d.txt" $i)
        mv CameraTrajectory.txt $(printf "CameraTrajectory_%d.txt" $i)
        mv KeyFrameTrajectory.txt $(printf "KeyFrameTrajectory_%d.txt" $i)

        sleep 2
    done

    # copy the settings 
    cp $MAIN_DIR/Examples$SUFFIX/RGB-D/$TUM_YAML . 

    cd ../
    mv $DATASET results/$DATASET

done < $DATASET_LIST_PATH

python generate_results.py 2>&1 | tee ./results/Stats.txt

echo moving results to ouput folder $OUTPUT_FOLDER
mv results $OUTPUT_FOLDER

xmessage -fn -*-fixed-*-*-*-*-40-*-*-*-*-*-iso8859-* 'rgbd_tum benchmarking completed'
