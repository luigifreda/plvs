#!/usr/bin/env bash

# clean all

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd ) # get script dir 
SCRIPT_DIR=$(readlink -f $SCRIPT_DIR)  # this reads the actual path if a symbolic directory is used
cd $SCRIPT_DIR # this brings us in the actual used folder (not the symbolic one)


CLEAN_OPENCV=0   # disabled, maybe you don't want to recompile opencv ;-)
if [ $CLEAN_OPENCV -eq 1 ] && [ -d Thirdparty/opencv ]; then
	echo "cleaning Thirdparty/opencv ..."
	cd Thirdparty
	rm -Rf opencv
	cd $SCRIPT_DIR	
fi

echo "cleaning Vocabulary"
if [[ -d Vocabulary ]]; then 
	rm "Vocabulary/ORBvoc.bin"
	cd $SCRIPT_DIR
fi

echo "cleaning Thirdparty/Pangolin ..."
if [[ -d Thirdparty/Pangolin ]]; then 
	cd Thirdparty/Pangolin
	if [[ -d build ]]; then 	
		rm -R build
	fi 
	cd $SCRIPT_DIR
fi 

echo "cleaning Thirdparty/rerun ..."
if [[ -d Thirdparty/rerun ]]; then 
	cd Thirdparty/rerun
	if [[ -d build ]]; then 	
		rm -Rf build install 
	fi 
	cd $SCRIPT_DIR
fi 

echo "cleaning Thirdparty/DBoW2 ..."
if [[ -d Thirdparty/DBoW2 ]]; then 
	cd Thirdparty/DBoW2
	if [[ -d build ]]; then 	
		rm -R build lib
	fi 	
	cd $SCRIPT_DIR
fi 

echo "cleaning Thirdparty/g2o ..."
if [[ -d Thirdparty/g2o ]]; then 
	cd Thirdparty/g2o
	if [[ -d build ]]; then 	
		rm -R build lib
	fi 	
	cd $SCRIPT_DIR
fi 

echo "cleaning Thirdparty/g2o_new ..."
if [ -d Thirdparty/g2o_new/build ]; then
	cd Thirdparty/g2o_new/build
	make clean
	cd ..
	rm -R build install 
	cd $SCRIPT_DIR
fi


echo "cleaning Thirdparty/volumetric_mapping ..."
if [ -d Thirdparty/volumetric_mapping ]; then
	cd Thirdparty/volumetric_mapping
	if [[ -d build ]]; then 	
		rm -R build lib
	fi 	
	cd $SCRIPT_DIR
fi 


echo "cleaning Thirdparty/open_chisel ..."
if [ -d Thirdparty/open_chisel ]; then
	cd Thirdparty/open_chisel
	if [[ -d build ]]; then 	
		rm -R build lib
	fi 	
	cd $SCRIPT_DIR
fi 


echo "cleaning Thirdparty/chisel_server ..."
if [ -d Thirdparty/chisel_server ]; then
	cd Thirdparty/chisel_server
	if [[ -d build ]]; then 	
		rm -R build lib
	fi 	
	cd $SCRIPT_DIR
fi 


echo "cleaning Thirdparty/fastfusion..."
cd Thirdparty/fastfusion
if [[ -d build ]]; then 	
	rm -R build lib
fi 	
cd $SCRIPT_DIR


echo "cleaning Thirdparty/voxblox ..."
cd Thirdparty/voxblox
if [[ -d build ]]; then 	
	rm -R build lib
fi 	
if [[ -f include/Block.pb.h ]]; then 
	rm include/*.pb.h
fi 
cd $SCRIPT_DIR


echo "cleaning Thirdparty/voxblox_server ..."
cd Thirdparty/voxblox_server
if [[ -d build ]]; then 	
	rm -R build lib
fi 	
cd $SCRIPT_DIR


echo "cleaning Thirdparty/libelas-gpu ..."
cd Thirdparty/libelas-gpu
if [[ -d build ]]; then 	
	rm -R build lib
fi 	
cd $SCRIPT_DIR


echo "cleaning Thirdparty/libsgm ..."
cd Thirdparty/libsgm
if [[ -d build ]]; then 	
	rm -R build lib
fi 	
cd $SCRIPT_DIR


echo "cleaning Thirdparty/line_descriptor..."
cd Thirdparty/line_descriptor
if [[ -d build ]]; then 	
	rm -R build lib
fi 	
cd $SCRIPT_DIR

./clean_plvs.sh 
