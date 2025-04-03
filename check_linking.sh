#!/usr/bin/env bash

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd ) # get script dir 
SCRIPT_DIR=$(readlink -f $SCRIPT_DIR)  # this reads the actual path if a symbolic directory is used
cd $SCRIPT_DIR # this brings us in the actual used folder (not the symbolic one)

function print_blue(){
    printf "\033[34;1m"
    printf "$@ \n"
    printf "\033[0m"
}

PROJECT_DIR=$SCRIPT_DIR

print_blue '================================================'

# check correct linking of opencv libs: you should see only one version of opencv 
cd $PROJECT_DIR
echo "Check you are getting only one version of opencv!"
ldd lib/libplvs.so | grep opencv

print_blue '================================================'

# check correct linking of opencv libs on ROS1 workspace: you should see only one version of opencv 
if [ -f ros_ws/devel/setup.bash ]; then 
    cd $PROJECT_DIR
    echo "Check you are getting only one version of opencv under ROS1!"    
    source ros_ws/devel/setup.bash 
    ldd ros_ws/devel/lib/plvs/RGBD | grep opencv  

    if [ -f ros_ws/devel/lib/libdepthai_ros_driver.so ]; then 
        echo "Check you are getting only one version of libdepthai_ros_driver under ros!"    
        ldd ros_ws/devel/lib/libdepthai_ros_driver.so | grep opencv          
    fi 
fi 

print_blue '================================================'

# check correct linking of opencv libs on ROS2 workspace: you should see only one version of opencv 
if [ -f ros2_ws/install/local_setup.bash ]; then 
    cd $PROJECT_DIR
    echo "Check you are getting only one version of opencv under ROS2!"    
    source ros2_ws/install/local_setup.bash 
    ldd ros2_ws/install/plvs/lib/plvs/rgbd | grep opencv
fi 

print_blue '================================================'

# check correct linking of g2o libs: you should see the libs  
cd $PROJECT_DIR
echo "Check you are getting the local g2o libs"
ldd lib/libplvs.so | grep g2o



print_blue '================================================'

# check correct linking of libpcl libs: you should see the libs  
cd $PROJECT_DIR
echo "Check you are getting the local pcl libs or only one version of it"
ldd lib/libplvs.so | grep pcl

