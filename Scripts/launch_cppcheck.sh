#!/bin/bash 

# required: 
# sudo apt install cppcheck

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd ) # get script dir 
SCRIPT_DIR=$(readlink -f $SCRIPT_DIR)  # this reads the actual path if a symbolic directory is used
cd $SCRIPT_DIR # this brings us in the actual used folder (not the symbolic one)


OUTPUT_LOG=$SCRIPT_DIR/Results/logCppCheck.txt

if [ ! -d Results ]; then 
	mkdir Results 
fi 

PROJECT_PATH="$SCRIPT_DIR/../"
echo "analysing code in $PROJECT_PATH"
cppcheck -v -j 4 --enable=all -ibuild -itest $PROJECT_PATH/src $PROJECT_PATH/include 2>&1 | tee $OUTPUT_LOG


