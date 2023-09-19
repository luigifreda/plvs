#!/usr/bin/env bash

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd ) # get script dir (this should be the main folder directory of PLVS)
SCRIPT_DIR=$(readlink -f $SCRIPT_DIR)  # this reads the actual path if a symbolic directory is used
cd $SCRIPT_DIR # this brings us in the actual used folder (not the possibly symbolic one)

MAIR_DIR=$SCRIPT_DIR/..


function create_links(){
	files=$1
	for file in $files; do 
		file_basename=$(basename $file) 
		file_extension="${file_basename##*.}"
		file_noextension="${file_basename%.*}"			
		if [[ "$file" == *"Monocular-Inertial"* ]]; then 
			ln -sf --relative $file "Monocular-Inertial-"$file_noextension"."$file_extension	
		elif [[ "$file" == *"Monocular"* ]]; then 
			ln -sf --relative $file "Monocular-"$file_noextension"."$file_extension	
		elif [[ "$file" == *"RGB-D-Inertial"* ]]; then 
			ln -sf --relative $file "RGB-D-Inertial-"$file_noextension"."$file_extension
		elif [[ "$file" == *"RGB-D"* ]]; then 
			ln -sf --relative $file "RGB-D-"$file_noextension"."$file_extension								
		elif [[ "$file" == *"Stereo-Inertial"* ]]; then 
			ln -sf --relative $file "Stereo-Inertial-"$file_noextension"."$file_extension
		elif [[ "$file" == *"Stereo"* ]]; then 
			ln -sf --relative $file "Stereo-"$file_noextension"."$file_extension			
		else 
			ln -sf --relative $file $file_basename	
		fi 
	done 
}

files=$(find $MAIR_DIR/Examples -maxdepth 2 -name "*.yaml" -print)
cd $SCRIPT_DIR/new
create_links "$files" 

files=$(find $MAIR_DIR/Examples_old -maxdepth 2 -name "*.yaml" -print)
cd $SCRIPT_DIR/old
create_links "$files" 