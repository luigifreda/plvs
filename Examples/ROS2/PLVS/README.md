# PLVS_ROS2

This package is a ROS2 wrapper for PLVS2. 
This was inspired by the repository: https://github.com/zang09/ORB_SLAM3_ROS2

---

## Prerequisites

This package was tested under 
  - Ubuntu 20.04
  - ROS1 noetic
  - ROS2 foxy
  - OpenCV 4.2.0

Install required ROS2 package
```
$ sudo apt install ros-$ROS_DISTRO-vision-opencv && sudo apt install ros-$ROS_DISTRO-message-filters
```

---
## How to use

See the main [README](../../../README.md#ros-build) on how to build PLVS with ROS2 support. 

1. Source the workspace  
    ```
    $ source <PLVS_ROOT>/ros2_ws/install/local_setup.bash
    ```
2. Run PLVS 
    At present, there is only support for `MONO, STEREO, RGBD, STEREO-INERTIAL` modes.  
      - `MONO` mode  
        ```
        $ ros2 run plvs mono PATH_TO_VOCABULARY PATH_TO_YAML_CONFIG_FILE
        ```
      - `STEREO` mode  
        ```
        $ ros2 run plvs stereo PATH_TO_VOCABULARY PATH_TO_YAML_CONFIG_FILE BOOL_RECTIFY
        ```
      - `RGBD` mode  
        ```
        $ ros2 run plvs rgbd PATH_TO_VOCABULARY PATH_TO_YAML_CONFIG_FILE
        ```
      - `STEREO-INERTIAL` mode  
        ```
        $ ros2 run plvs stereo-inertial PATH_TO_VOCABULARY PATH_TO_YAML_CONFIG_FILE BOOL_RECTIFY [BOOL_EQUALIZE]
        ```

---
## Run with ros1 bags via ros bridge

To play ros1 bag file, you should install `ros1 noetic` & `ros1 bridge`.  

Here is a [link](https://www.theconstructsim.com/ros2-qa-217-how-to-mix-ros1-and-ros2-packages/) to demonstrate example of `ros1-ros2 bridge` procedure.  
If you have `ros1 noetic` and `ros1 bridge` already, open your terminal and follow this:  
(Shell A, B, C, D is all different terminal, e.g. `stereo-inertial` mode)
1. Download EuRoC Dataset (`V1_02_medium.bag`)
    ```
    $ wget -P ~/Downloads http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/vicon_room1/V1_02_medium/V1_02_medium.bag
    ```  

2. Launch Terminal  
  (e.g. `ROS1_INSTALL_PATH`=`/opt/ros/noetic`, `ROS2_INSTALL_PATH`=`/opt/ros/foxy`)
    ```
    #Shell A:
    source ${ROS1_INSTALL_PATH}/setup.bash
    roscore

    #Shell B:
    source ${ROS1_INSTALL_PATH}/setup.bash
    source ${ROS2_INSTALL_PATH}/setup.bash
    export ROS_MASTER_URI=http://localhost:11311
    ros2 run ros1_bridge dynamic_bridge

    #Shell C:
    source ${ROS1_INSTALL_PATH}/setup.bash
    rosbag play ~/Downloads/V1_02_medium.bag --pause /cam0/image_raw:=/camera/left /cam1/image_raw:=/camera/right /imu0:=/imu

    #Shell D:
    source ${ROS2_INSTALL_PATH}/setup.bash
    ros2 run plvs stereo-inertial PATH_TO_VOCABULARY PATH_TO_YAML_CONFIG_FILE BOOL_RECTIFY [BOOL_EQUALIZE]
    ```

3. Press `spacebar` in `Shell C` to resume bag file.  
