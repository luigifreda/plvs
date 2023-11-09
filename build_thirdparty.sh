#!/usr/bin/env bash

. config.sh  # source configuration file and utils 

# ====================================================

print_blue '================================================'
print_blue "Building Thirdparty"
print_blue '================================================'

set -e

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd ) # get script dir (this should be the main folder directory of PLVS)
SCRIPT_DIR=$(readlink -f $SCRIPT_DIR)  # this reads the actual path if a symbolic directory is used
cd $SCRIPT_DIR # this brings us in the actual used folder (not the possibly symbolic one)
#echo "current dir: $SCRIPT_DIR"

UBUNTU_VERSION=$(lsb_release -a 2>&1)  # to get ubuntu version 

# ====================================================
# check if we have external options
EXTERNAL_OPTION=$1
if [[ -n "$EXTERNAL_OPTION" ]]; then
    echo "external option: $EXTERNAL_OPTION" 
fi

# check if we set a C++ standard
if [[ -n "$CPP_STANDARD_VERSION" ]]; then
    echo "CPP_STANDARD_VERSION: $CPP_STANDARD_VERSION" 
    EXTERNAL_OPTION="$EXTERNAL_OPTION -DCPP_STANDARD_VERSION=$CPP_STANDARD_VERSION"
fi

# check the use of local opencv
if [[ -n "$OpenCV_DIR" ]]; then
    echo "OpenCV_DIR: $OpenCV_DIR" 
    EXTERNAL_OPTION="$EXTERNAL_OPTION -DOpenCV_DIR=$OpenCV_DIR"
fi

# check CUDA options
if [ $USE_CUDA -eq 1 ]; then
    echo "USE_CUDA: $USE_CUDA" 
    EXTERNAL_OPTION="$EXTERNAL_OPTION -DWITH_CUDA=ON"
fi

if [[ $OPENCV_VERSION == 4* ]]; then
    EXTERNAL_OPTION="$EXTERNAL_OPTION -DOPENCV_VERSION=4"
fi

 echo "external option: $EXTERNAL_OPTION"
# ====================================================

# install ubuntu dependancies!
#./install_dependencies.sh

# ====================================================

print_blue '================================================'
print_blue "Configuring and building Thirdparty/Pangolin ..."

cd Thirdparty
if [ ! -d Pangolin ]; then
	sudo apt-get install -y libglew-dev
	git clone  --recursive https://github.com/stevenlovegrove/Pangolin.git
    #git fetch --all --tags # to fetch tags 
    cd Pangolin
    #git checkout tags/v0.6
    git checkout fe57db532ba2a48319f09a4f2106cc5625ee74a9
    git apply ../pangolin.patch  # applied to commit fe57db532ba2a48319f09a4f2106cc5625ee74a9
    cd .. 
fi
cd Pangolin
make_buid_dir
if [[ ! -f build/src/libpangolin.so && ! -f build/libpango_core.so ]]; then
	cd build
	#cmake .. -DCMAKE_BUILD_TYPE=Release -DAVFORMAT_INCLUDE_DIR="" -DCPP11_NO_BOOST=ON $EXTERNAL_OPTION
	cmake .. -DCMAKE_INSTALL_PREFIX="`pwd`/../install" -DCMAKE_BUILD_TYPE=Release $EXTERNAL_OPTION    
	make -j 8
        make install     
fi
cd $SCRIPT_DIR

print_blue '================================================'
print_blue "Configuring and building Thirdparty/DBoW2 ..."

cd Thirdparty/DBoW2
make_buid_dir
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release $EXTERNAL_OPTION
make -j 8
cd $SCRIPT_DIR

print_blue '================================================'
print_blue "Configuring and building Thirdparty/g2o ..."

cd Thirdparty/g2o
if [ ! -d build ]; then
    sudo apt-get install -y libsuitesparse-dev libeigen3-dev
fi
make_buid_dir
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release $EXTERNAL_OPTION
make -j 8
cd $SCRIPT_DIR

./install_local_g2o_new.sh $EXTERNAL_OPTION

print_blue '================================================'
print_blue "Configuring and building Thirdparty/volumetric_mapping ... "

cd Thirdparty/volumetric_mapping
if [ ! -d build ]; then
    sudo apt-get install -y libgoogle-glog-dev
fi
make_buid_dir
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release $EXTERNAL_OPTION
make -j 8
cd $SCRIPT_DIR

print_blue '================================================'
print_blue "Configuring and building Thirdparty/open_chisel ... "

cd Thirdparty/open_chisel
make_buid_dir
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release $EXTERNAL_OPTION
make -j 8
cd $SCRIPT_DIR

print_blue '================================================'
print_blue "Configuring and building Thirdparty/chisel_server ... "

cd Thirdparty/chisel_server
make_buid_dir
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release $EXTERNAL_OPTION
make -j 8
cd $SCRIPT_DIR

print_blue '================================================'
print_blue "Configuring and building Thirdparty/fastfusion ... "

HAVE_SSE4=$(gcc -march=native -dM -E - </dev/null | grep SSE4 || :)
HAVE_AVX=$(gcc -march=native -dM -E - </dev/null | grep AVX || : )
if [[ -n "$HAVE_SSE4" ]] || [[ -n "$HAVE_AVX" ]] ; then
	echo "Configuring and building Thirdparty/fastfusion ... "
	cd Thirdparty/fastfusion
	make_buid_dir
	cd build
	cmake .. -DCMAKE_BUILD_TYPE=Release $EXTERNAL_OPTION
	make -j 8
fi
cd $SCRIPT_DIR

print_blue '================================================'
print_blue "Configuring and building Thirdparty/voxblox ... "

cd Thirdparty/voxblox
if [ ! -d build ]; then
    sudo apt-get install -y libprotobuf-dev protobuf-compiler
fi
make_buid_dir
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release $EXTERNAL_OPTION
make -j 8
# HACK (to be fixed inside cmake) copy the protobuffer files in the include folder
if [ ! -f Thirdparty/voxblox/include/Block.pb.h ] && [ ! -f Thirdparty/voxblox/include/Layer.pb.h ] ; then
    cmake .. -DCMAKE_BUILD_TYPE=Release $EXTERNAL_OPTION
fi
cd $SCRIPT_DIR

print_blue '================================================'
print_blue "Configuring and building Thirdparty/voxblox_sever ... "

cd Thirdparty/voxblox_server
make_buid_dir
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release $EXTERNAL_OPTION
make -j 8
cd $SCRIPT_DIR

print_blue '================================================'

if [[ -n "$HAVE_SSE3" ]]; then
    echo "Configuring and building Thirdparty/libelas-gpu ... "
    cd Thirdparty/libelas-gpu
    make_buid_dir
    cd build
    cmake .. -DCMAKE_BUILD_TYPE=Release $EXTERNAL_OPTION
    make -j 8
fi
cd $SCRIPT_DIR

print_blue '================================================'

if [ $CUDA_FOUND -eq 1 ]; then
    echo "Configuring and building Thirdparty/stereo_libsgm... "
    
    cd Thirdparty/libsgm
    if [ ! -d build ]; then 
        if [[ $UBUNTU_VERSION == *"18.04"* ]] ; then
            sudo apt-get install -y libglfw3-dev
        fi
        if [[ $UBUNTU_VERSION == *"20.04"* ]] ; then
            sudo apt-get install -y libglfw3-dev
            sudo apt-get install -y libxxf86vm-dev
        fi
    fi 
    make_buid_dir
    cd build
    cmake .. -DCMAKE_BUILD_TYPE=Release $EXTERNAL_OPTION
    make -j 8
fi
cd $SCRIPT_DIR

print_blue '================================================'
print_blue "Configuring and building Thirdparty/line_descriptor ... "

cd Thirdparty/line_descriptor
make_buid_dir
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release $EXTERNAL_OPTION
make -j 8
cd $SCRIPT_DIR

print_blue '================================================'
print_blue "Checking pip and evo package for odometry performance evaluation... "

DO_INSTALL_PIP=$(check_package python3-pip)
if [ $DO_INSTALL_PIP -eq 1 ] ; then
	echo "installing pip"
	sudo apt-get install -y python3-pip
fi	
# original package from https://github.com/MichaelGrupp/evo
#                       https://github.com/MichaelGrupp/evo/wiki/evo_traj
DO_INSTALL_EVO=$(check_pip_package evo)
if [ $DO_INSTALL_EVO -eq 1 ] ; then
	echo "installing evo package"
	pip install evo --upgrade --no-binary evo --user
fi



