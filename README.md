# PLVS II

### v0.2.1

Author: [Luigi Freda](https://www.luigifreda.com)


PLVS is a real-time system that combines sparse SLAM, volumetric mapping, and 3D unsupervised incremental segmentation. PLVS stands for **Points**, **Lines**, **Volumetric mapping**, and **Segmentation**. 

<p align="center">
<img src="Images/PLVS-lab.png"
alt="PLVS lab" max-width="585" border="0"/> 
<img src="Images/PLVS-ar2.gif"
alt="PLVS augmented reality" height="180" border="0"/> 
<img src="Images/PLVS-details.gif"
alt="PLVS details" height="180" border="0"/> 
<img src="Images/PLVS-Points-Lines-Vol-Seg.png"
alt="PLVS details" max-width="695" border="0"/> 
</p>

PLVS is available in two different versions.
- **PLVS I**: hosted in the branch `plvs1`. It builds on [ORB-SLAM2](https://github.com/raulmur/ORB_SLAM2), and supports mono, stereo, and RGB-D cameras.
- **PLVS II**: hosted in the `master` branch. It builds on [ORB-SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3), and also supports camera systems provided with IMUs. 

This [document](./new_features.md) provides a list of the **new features** introduced by PLVS. For more information and videos, please visit this [project page](https://www.luigifreda.com/research/plvs-an-open-source-rgb-d-and-stereo-slam-for-volumetric-reconstruction-and-3d-incremental-segmentation/) or refer to the following **document**:
**[PLVS: A SLAM System with Points, Lines, Volumetric Mapping, and 3D Incremental Segmentation](https://arxiv.org/pdf/2309.10896.pdf)**         
*Luigi Freda* 

**Note**: PLVS is an active project. This *README* is under construction and will be updated with further information and details as new improvements are released. Stay tuned!

  
## Quick start 

- The [build](#build) procedures were tested under **Ubuntu 20, 22, 24**. 
- ROS1 support is provided only under **noetic** with **Ubuntu 20** (see [here](#ros-build)).
- ROS2 was tested under **foxy** and **Ubuntu 20** (further details [here](#ros-build)). 

If you don't have Ubuntu 20 with noetic, check [rosdocker](https://github.com/luigifreda/rosdocker) and use the *noetic* or *noetic_cuda* containers.

**NOTE**: At present, under Ubuntu 24.04, `BUILD_WITH_MARCH_NATIVE` is set to `OFF`. Enabling `--march=native` optimization brings some problems probably due to different default building options in the native `libpcl`.

---
### Build

1. Install basic dependencies:      
  `$ ./install_dependencies.sh`        
2. Install *OpenCV* in a local folder:                 
  `$ ./install_local_opencv.sh`         
3. Build the *PLVS* framework:       
  `$ ./build.sh`

It should be easy to adapt the above procedures if you have a different OS version. 

If you want to skip step 2, you can set the variables `OpenCV_DIR` and `OPENCV_VERSION` in `config.sh` with your local *OpenCV* path and version, respectively. However, this is not recommended. 

### ROS build

#### ROS 1

Under **ROS noetic**, open a new terminal, source the main ROS1 `setup.bash` and run:        
`$ ./build_ros_catkin.sh`       

This command builds the *PLVS* ROS1 workspace in `Example_old/ROS/PLVS` and deploys it in the `ros_ws` folder.

#### ROS 2

Under **ROS 2**, open a new terminal, source the main ROS2 `setup.bash` and run:      
`$ ./build_ros_colcon.sh`       

This command builds the *PLVS* ROS2 workspace in `Example/ROS2/PLVS` and deploys it in the `ros2_ws` folder.


---
### Running the examples 

Once everything is built, you can enter in the `Scripts` folder and test the different examples. For instance you can configure and run: 
- `$ ./run_tum_rgbd.sh` for TUM RGB-D datasets 
- `$ ./run_kitti_stereo.sh` for KITTI datasets
- `$ ./run_euroc_stereo_inertial.sh` for Euroc datasets, stereo + inertial
- `$ ./run_euroc_stereo.sh` for Euroc datasets, only stereo
- `$ ./run_tum_vi_stereo.sh` for TUM VI datasets, only stereo
- `$ ./run_tum_vi_stereo_inertial.sh` for TUM VI datasets, stereo + inertial

In each of the above scripts, you have to configure *(1)* the `DATASET_BASE_FOLDER`, *(2)* the specific `DATASET` of interest, and *(3)* the used `YAML` configuration file. In particular, each `YAML` configuration file shows different sections with commented options. For a quick overview of the **new features** and their corresponding `YAML` options refer to [new_features.md](./new_features.md).   

#### ROS 1 

If you built the ROS workspace, you can use the scripts `ros_xterm*` to launch the PLVS ROS nodes. 
For instance, with the TUM datasets, configure and run `ros_xterm_tum_rgbd.sh`.

#### ROS 2

Refer to this [README](./Examples/ROS2/PLVS/README.md). At present, this is a work in progress.
With the TUM datasets, configure and run `ros2_xterm_tum_rgbd.sh`.

**Note**: The ROS1 and ROS2 install paths are automatically detected by the script `Scripts/find_ros.sh`.

---
## Contributing

We welcome contributions to the codebase through pull requests, bug reports, comments, and feature proposals via issues. For any questions or feedback, please contact *luigifreda(at)gmail(dot)com*. Thank you!

---
## License 

PLVS is released under [GPLv3 license](./LICENSE). PLVS contains some modified libraries, each one coming with its license. Where nothing is specified, a GPLv3 license applies to the software.

If you use PLVS in your projects, please cite our above-mentioned document.

---
## Credits  

* The *PLVS I* and *PLVS II* frameworks are based on [ORB-SLAM2](https://github.com/raulmur/ORB_SLAM2) and [ORB-SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3) respectively. Many thanks to their Authors for their great work and contributions to the Research and open-source communities. 
* Ther ROS 2 wrapper was inspired by this [repository](https://github.com/zang09/ORB_SLAM3_ROS2). Many thanks to his Author, [Haebeom Jung](https://github.com/zang09). 

