#!/usr/bin/env bash

echo "Installing main dependencies ..."

set -e

# generic
sudo apt-get update
sudo apt-get install -y build-essential cmake   
sudo apt-get install -y libeigen3-dev 
sudo apt-get install -y libopenni-dev libopenni2-dev libpcl-dev
sudo apt-get install -y curl software-properties-common

#sudo apt-get install -y libzstd-devel 

# pangolin
sudo apt-get install -y libglew-dev libglfw3 libglfw3-dev
sudo apt-get install ffmpeg -y libavcodec-dev libavutil-dev libavformat-dev libswscale-dev libavdevice-dev
sudo apt-get install libjpeg-dev -y libpng-dev libtiff5-dev libopenexr-dev

# g2o new
sudo apt-get install -y libsuitesparse-dev

# voxblox and volumetric mapping 
sudo apt-get install -y libgoogle-glog-dev
sudo apt-get install -y libprotobuf-dev protobuf-compiler
sudo apt-get install -y libpcl-conversions-dev liboctomap-dev

# python for rgbd-tum dataset tools and evo package
sudo apt-get install -y python3-pip python3-scipy python3-sklearn-pandas

# realsense 
# see https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md#installing-the-packages
sudo apt-get install -y apt-transport-https
if [ ! -d /etc/apt/keyrings ]; then 
    sudo mkdir -p /etc/apt/keyrings
fi 
if [ ! -f /etc/apt/sources.list.d/librealsense.list ]; then 
    curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null
    echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo `lsb_release -cs` main" | \
    sudo tee /etc/apt/sources.list.d/librealsense.list
    sudo apt-get update
fi 
sudo apt-get install -y librealsense2-dkms librealsense2-utils
sudo apt-get install -y librealsense2-dev librealsense2-dbg

# ros 
# see https://catkin-tools.readthedocs.io/en/latest/installing.html
# sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" \
#         > /etc/apt/sources.list.d/ros-latest.list'
# wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
# sudo apt-get install -y python3-catkin-tools
sudo pip3 install -U catkin_tools

echo "...All deps installed!"
