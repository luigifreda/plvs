#!/usr/bin/env bash

. config.sh  # source configuration file and utils 


set -e

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd ) # get script dir (this should be the main folder directory of PLVS)
SCRIPT_DIR=$(readlink -f $SCRIPT_DIR)  # this reads the actual path if a symbolic directory is used
cd $SCRIPT_DIR # this brings us in the actual used folder (not the possibly symbolic one)


EXTERNAL_OPTION=$1
if [[ -n "$EXTERNAL_OPTION" ]]; then
    echo "external option: $EXTERNAL_OPTION" 
fi

print_blue '================================================'
print_blue "Configuring and building Thirdparty/pcl ..."

cd Thirdparty
if [ ! -d pcl ]; then
	git clone https://github.com/PointCloudLibrary/pcl.git pcl
    #git fetch --all --tags # to fetch tags 
    cd pcl
    git checkout tags/pcl-1.10.0   
    #git apply ../pcl.patch 
    cd .. 
fi
cd pcl
make_buid_dir
if [[ ! -f install/lib/libpcl_common.so ]]; then
	cd build
    PCL_OPTIONS="-DWITH_CUDA=FALSE" 
    echo "compiling with options: $PCL_OPTIONS $EXTERNAL_OPTION" 
    cmake .. -DCMAKE_INSTALL_PREFIX="`pwd`/../install" -DCMAKE_BUILD_TYPE=Release $PCL_OPTIONS $EXTERNAL_OPTION
	make -j 8
    make install 
fi 
cd $SCRIPT_DIR