EVALUATE_SCRIPT="./evaluate_ate.py"

TRAJECTORY_PATH=`pwd`

GROUDTRUTH_PATH=$1
if [[ -n "$GROUDTRUTH_PATH" ]] 
then
	echo GROUDTRUTH_PATH: $GROUDTRUTH_PATH
else
	echo missing groundtruth
	exit 1
fi

echo TRAJECTORY_PATH=$TRAJECTORY_PATH
echo GROUDTRUTH_PATH=$GROUDTRUTH_PATH

python $EVALUATE_SCRIPT --verbose $GROUDTRUTH_PATH/groundtruth.txt $TRAJECTORY_PATH/CameraTrajectory.txt --save_transformation transformation.txt --plot CameraTrajectory.png
python $EVALUATE_SCRIPT --verbose $GROUDTRUTH_PATH/groundtruth.txt $TRAJECTORY_PATH/KeyFrameTrajectory.txt --save_transformation transformation.txt --plot KeyFrameTrajectory.png
