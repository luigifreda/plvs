#!/usr/bin/env bash

reset 

. config.sh  # source configuration file 

# ====================================================

print_blue '================================================'
print_blue 'Building PLVS'
print_blue '================================================'

set -e

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd ) # get script dir 
SCRIPT_DIR=$(readlink -f $SCRIPT_DIR)  # this reads the actual path if a symbolic directory is used
cd $SCRIPT_DIR # this brings us in the actual used folder (not the symbolic one)
#echo "current dir: $SCRIPT_DIR"

UBUNTU_VERSION=$(lsb_release -a 2>&1)  # to get ubuntu version 

# ====================================================
# check if we have external options
EXTERNAL_OPTIONS=$1
if [[ -n "$EXTERNAL_OPTIONS" ]]; then
    echo "external options: $EXTERNAL_OPTIONS" 
fi

# check if we set a BUILD_TYPE
if [[ -n "$BUILD_TYPE" ]]; then
    echo "BUILD_TYPE: $BUILD_TYPE" 
    EXTERNAL_OPTIONS="$EXTERNAL_OPTIONS -DCMAKE_BUILD_TYPE=$BUILD_TYPE"
else
    echo "setting BUILD_TYPE to Release by default"
    EXTERNAL_OPTIONS="$EXTERNAL_OPTIONS -DCMAKE_BUILD_TYPE=Release"     
fi

# check if we set BUILD_WITH_MARCH_NATIVE
if [[ -n "$BUILD_WITH_MARCH_NATIVE" ]]; then
    echo "BUILD_WITH_MARCH_NATIVE: $BUILD_WITH_MARCH_NATIVE" 
    EXTERNAL_OPTIONS="$EXTERNAL_OPTIONS -DBUILD_WITH_MARCH_NATIVE=$BUILD_WITH_MARCH_NATIVE"
fi

# check if we set a C++ standard
if [[ -n "$CPP_STANDARD_VERSION" ]]; then
    echo "CPP_STANDARD_VERSION: $CPP_STANDARD_VERSION" 
    EXTERNAL_OPTIONS="$EXTERNAL_OPTIONS -DCPP_STANDARD_VERSION=$CPP_STANDARD_VERSION"
fi

# check the use of local opencv
if [[ -n "$OpenCV_DIR" ]]; then
    echo "OpenCV_DIR: $OpenCV_DIR" 
    EXTERNAL_OPTIONS="$EXTERNAL_OPTIONS -DOpenCV_DIR=$OpenCV_DIR"
fi

# check CUDA options
if [ $USE_CUDA -eq 1 ]; then
    echo "USE_CUDA: $USE_CUDA" 
    EXTERNAL_OPTIONS="$EXTERNAL_OPTIONS -DWITH_CUDA=ON"
fi

if [[ $OPENCV_VERSION == 4* ]]; then
    EXTERNAL_OPTIONS="$EXTERNAL_OPTIONS -DOPENCV_VERSION=4"
fi

if [[ "$UBUNTU_VERSION" == *"24.04"* ]] ; then
	# At present, the standard g2o version generates some crashes under ubuntu 24.04
    echo "We force the use of the new g2o version!"
    EXTERNAL_OPTIONS="$EXTERNAL_OPTIONS -DWITH_G2O_NEW=ON"
fi 

 echo "external options: $EXTERNAL_OPTIONS"
# ====================================================

echo "Configuring and building PLVS ..."

if [ ! -d build ]; then
    mkdir build
fi
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release $EXTERNAL_OPTIONS
make -j6   # if you use too many threads your will loose the control of your computer for a while 
