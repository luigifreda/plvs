#!/usr/bin/env bash

. config.sh  # source configuration file and utils 

set -e

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd ) # get script dir (this should be the main folder directory of PLVS)
SCRIPT_DIR=$(readlink -f $SCRIPT_DIR)  # this reads the actual path if a symbolic directory is used
cd $SCRIPT_DIR # this brings us in the actual used folder (not the possibly symbolic one)


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

echo "external options: $EXTERNAL_OPTIONS"

print_blue '================================================'
print_blue "Configuring and building Thirdparty/g2o_new ..."

cd Thirdparty
if [ ! -d g2o_new ]; then
    sudo apt install -y libqglviewer-dev-qt5  # to build g2o_viewer 
    git clone https://github.com/RainerKuemmerle/g2o.git g2o_new
    #git fetch --all --tags # to fetch tags 
    cd g2o_new
    git checkout tags/20230223_git   
    git apply ../g2o.patch 
    cd .. 
fi
cd g2o_new
make_buid_dir
if [[ ! -f install/lib/libg2o_core.so ]]; then
	cd build
    #G2O_OPTIONS="-DBUILD_WITH_MARCH_NATIVE=ON -DG2O_BUILD_EXAMPLES=OFF"  # march native controlled by the above option!
    G2O_OPTIONS="-DG2O_BUILD_EXAMPLES=OFF" 
    echo "compiling with options: $G2O_OPTIONS $EXTERNAL_OPTIONS" 
    cmake .. -DCMAKE_INSTALL_PREFIX="`pwd`/../install" -DCMAKE_BUILD_TYPE=Release $G2O_OPTIONS $EXTERNAL_OPTIONS
	make -j 8
    make install 
fi 
cd $SCRIPT_DIR
