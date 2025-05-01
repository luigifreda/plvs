#!/usr/bin/env bash

source bash_utils.sh

CONFIG_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
CONFIG_DIR=$(readlink -f $CONFIG_DIR)  # this reads the actual path if a symbolic directory is used
cd $CONFIG_DIR # this brings us in the actual folder of this config script (not the symbolic one)
#echo "current dir: $CONFIG_DIR"

# ====================================================
# C++ standard  
# ====================================================

export CPP_STANDARD_VERSION=17   # we need c++17 since nvcc does not support c++20 yet (probably we can try mixing c++ standards and just let nvcc use c++17 ... not sure this is the best choice)

# ====================================================
# OpenCV Settings 
# ====================================================

# 1: ON, 0: OFF
export USE_LOCAL_OPENCV=1   # use a local installation of OpenCV

# or you can set manullay OpenCV_DIR
# export OpenCV_DIR="path to my OpenCV folder"
# export OpenCV_DIR  # here not set 

export OPENCV_VERSION=4

# ====================================================
# CUDA Settings
# ====================================================

# N.B: if you do not have installed opencv with CUDA support you must set above:
# USE_LOCAL_OPENCV=1

# 1: ON, 0: OFF
export USE_CUDA=0  # use CUDA in PLVS sparse SLAM  
export CUDA_VERSION="cuda"  # must be an installed CUDA path in "/usr/local"; 
                                 # if available, you can use the simple path "/usr/local/cuda" which should be a symbolic link to the last installed cuda version 
if [ ! -d /usr/local/$CUDA_VERSION ]; then
    CUDA_VERSION="cuda"  # use last installed CUDA path in standard path as a fallback 
fi 

export PATH=/usr/local/$CUDA_VERSION/bin${PATH:+:${PATH}}
export LD_LIBRARY_PATH=/usr/local/$CUDA_VERSION/lib64${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}
export CUDADIR=/usr/local/$CUDA_VERSION

# ====================================================
# Dataset Settings 
# ====================================================

# to be used in run/xterm scripts
export DATASETS_FOLDER="~/Work/datasets/rgbd_datasets/tum/"

# ====================================================
# ROS Settings 
# ====================================================

export INSTALL_CATKIN_TOOLS=1  # now this is compulsory

export USE_ZED_CAMERA=0  # ( you need to install the last zed camera SDK)
export USE_REALSENSE_D435=0
export USE_ELAS_ROS=0


# ====================================================
# Check and Manage Settings 
# ====================================================

# auto-managed things below ...


# ====================================================
# SIMD

# check SIMD supports 
export HAVE_SSE3=$(gcc -march=native -dM -E - </dev/null | grep SSE3 || :)
export HAVE_SSE4=$(gcc -march=native -dM -E - </dev/null | grep SSE4 || :)
export HAVE_AVX=$(gcc -march=native -dM -E - </dev/null | grep AVX || : )

# ====================================================
# CUDA 

export CUDA_FOUND=0
if [ -f /usr/local/$CUDA_VERSION/bin/nvcc ] || [ -f /usr/bin/nvcc ]; then
	CUDA_FOUND=1
	echo "CUDA found: $CUDA_VERSION"
fi

# reset env var if CUDA lib is not installed 
if [ $CUDA_FOUND -eq 0 ]; then
	USE_CUDA=0
	echo 'CUDA env var reset, check your CUDA installation'
fi

# ====================================================
# OPENCV 

if [[ -n "$OpenCV_DIR" ]]; then
	if [ ! -d $OpenCV_DIR ]; then 
		echo OpenCV_DIR does not exist: $OpenCV_DIR
		exit 1 
	fi 
fi 

# install a local opencv with CUDA support and more
if [ $USE_LOCAL_OPENCV -eq 1 ] && [[ ! -n "$OpenCV_DIR" ]]; then
	. install_local_opencv.sh   # source it in order to run it and get the env var OPENCV_VERSION
	echo OpenCV version: $OPENCV_VERSION
	if [[ $OPENCV_VERSION == 4* ]]; then
		OpenCV_DIR="$CONFIG_DIR/Thirdparty/opencv/install/lib/cmake/opencv4"
	else
		OpenCV_DIR="$CONFIG_DIR/Thirdparty/opencv/install/share/OpenCV"
	fi
	echo setting OpenCV_DIR: $OpenCV_DIR
    #export LD_LIBRARY_PATH=$CONFIG_DIR/Thirdparty/opencv/install/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}
fi

# ====================================================
# ZED 

export ZED_INSTALL_FOUND=0
if [ -d /usr/local/zed ]; then
	ZED_INSTALL_FOUND=1
	echo "ZED found"
fi

# reset env var if ZED is not installed 
if [ $ZED_INSTALL_FOUND -eq 0 ]; then
	USE_ZED_CAMERA=0
	echo 'ZED env var reset, check your ZED installation'
fi

