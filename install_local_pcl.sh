#!/usr/bin/env bash

. config.sh  # source configuration file and utils 


set -e

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd ) # get script dir (this should be the main folder directory of PLVS)
SCRIPT_DIR=$(readlink -f $SCRIPT_DIR)  # this reads the actual path if a symbolic directory is used
cd $SCRIPT_DIR # this brings us in the actual used folder (not the possibly symbolic one)

UBUNTU_VERSION=$(lsb_release -a 2>&1)  # ubuntu version


# See the tags available here https://github.com/PointCloudLibrary/pcl/tags
# To minimize issues, let's use the latest stable version in the current ubuntu version
PCL_VERSION="1.10.0"
if [[ "$UBUNTU_VERSION" == *"22.04"* ]] ; then
    PCL_VERSION="1.12.0"
fi
if [[ "$UBUNTU_VERSION" == *"24.04"* ]] ; then
    PCL_VERSION="1.14.0"
fi

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
    if [[ "$BUILD_WITH_MARCH_NATIVE" == "ON" ]]; then
        if [[ "$PCL_VERSION" == "1.14"* ]] ; then    
            EXTERNAL_OPTIONS="$EXTERNAL_OPTIONS -DPCL_ENABLE_MARCHNATIVE=ON"   # PCL_VERSION="1.14.0"
        fi
        if [[ "$PCL_VERSION" == "1.12"* ]] ; then  
            EXTERNAL_OPTIONS="$EXTERNAL_OPTIONS -DPCL_ENABLE_SSE=ON"           # PCL_VERSION="1.12.0"
        fi          
        if [[ "$PCL_VERSION" == "1.10"* ]] ; then  
            EXTERNAL_OPTIONS="$EXTERNAL_OPTIONS -DPCL_ENABLE_SSE=ON"           # PCL_VERSION="1.10.0"
        fi 
    fi
fi

echo "external options: $EXTERNAL_OPTIONS"

print_blue '================================================'
print_blue "Configuring and building Thirdparty/pcl ..."

cd Thirdparty
if [ ! -d pcl ]; then
	git clone https://github.com/PointCloudLibrary/pcl.git pcl
    cd pcl
    git fetch --all --tags # to fetch tags 
    git checkout "pcl-$PCL_VERSION"
    #git apply ../pcl.patch 
    cd .. 
fi

cd pcl
make_buid_dir

if [[ ! -f install/lib/libpcl_common.so ]]; then
	cd build
    PCL_OPTIONS="-DWITH_CUDA=FALSE" 
    echo "compiling with options: $PCL_OPTIONS $EXTERNAL_OPTIONS" 
    cmake .. -DCMAKE_INSTALL_PREFIX="`pwd`/../install" -DCMAKE_BUILD_TYPE=Release $PCL_OPTIONS $EXTERNAL_OPTIONS
	make -j 8
    make install
    cd .. 
fi 

if [[ ! -d install/share/pcl ]]; then 
    cd install/share
    ln -sf pcl* pcl
    cd -
fi 
cd $SCRIPT_DIR