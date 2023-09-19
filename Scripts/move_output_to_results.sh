#!/bin/bash 

# move the generated output to the results folder


SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd ) # get script dir 
SCRIPT_DIR=$(readlink -f $SCRIPT_DIR)  # this reads the actual path if a symbolic directory is used
cd $SCRIPT_DIR # this brings us in the actual used folder (not the symbolic one)

DATASET_NAME=$1
RESULTS_DEST_FOLDER="$SCRIPT_DIR/Results/"

# create dest folder 
if [ ! -d $RESULTS_DEST_FOLDER ]; then 
	mkdir -p $RESULTS_DEST_FOLDER 
fi 

# keywords for the generated output files 
KEYWORDS=( \
"*CameraTrajectory*" \
"*KeyFrameTrajectory*" \
transformation.txt \
Performances.txt \
)

if [[ -n "$DATASET_NAME" ]]; then
  KEYWORDS+=("*"$DATASET_NAME"*")
fi 

for keyword in "${KEYWORDS[@]}"; do 
    # get all files in dir $SCRIPT_DIR that contains the keyword $keyword
    files=$(find $SCRIPT_DIR -maxdepth 1 -name "$keyword" -print)
    if [ -z "$files" ]; then
        continue
    fi    
    #echo moving files: $files 
    for file in $files; do 
        mv $file $RESULTS_DEST_FOLDER
    done
done 

echo Moved output to $RESULTS_DEST_FOLDER