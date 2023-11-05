# New Features 

This is a list of the **new features** provided by PLVS: 
* **Line segment** detection, matching, triangulation and tracking. 
  - This capability can be enabled via the option `Line.on` in the yaml settings.
  - Removed some bugs and optimized parts of the adopted [line_descriptor](https://github.com/opencv/opencv_contrib/tree/4.x/modules/line_descriptor) OpenCV module. 
* Dense reconstruction with different **volumetric mapping methods**: *voxelgrid*, *octree_point*, *[octomap](https://github.com/OctoMap/octomap)*, *[fastfusion](https://github.com/tum-vision/fastfusion)*, *[chisel](https://github.com/personalrobotics/OpenChisel)*, *[voxblox](https://github.com/ethz-asl/voxblox)*.  
  - It can be enabled by using the option `PointCloudMapping.on` in the yaml settings and selecting your preferred method `PointCloudMapping.type` (see the comments in the yaml files). 
* **Incremental segmentation** with RGBD sensors and octree-based dense map. 
  - It can be enabled by using the option `Segmentation.on` in the yaml settings of the RGBD cameras (only working when `octree_point` is selected as volumetric mapping method). 
* **Augmented reality** with overlay of tracked features, built meshes and loaded 3D models. 
  - This viz can be enabled by using the button `AR Camera` in the viewer GUI. 
* Generated **sparse and dense maps** can be **saved** and **reloaded**. 
  - You can save the generated sparse and dense maps anytime by using the GUI: first press the button `Pause` and then press the button `Save`. As a consequence, maps will be saved in the *Scripts* folder. In particular, *(1)* a sparse map will be always saved, *(2)* a dense map will be saved in the form of a ply (or of another custom format) only in the case you have set `PointCloudMapping.on: 1`. 
  - Use the `SparseMapping` options (as showed in this [TUM configuration file](./Settings/old/RGB-D-TUM1.yaml)) in order to reload the sparse map. In particular, be sure to set the `SparseMapping.filename` and then set `SparseMapping.reuseMap: 1`. 
  - As for reloading the dense map, set `PointCloudMapping.loadMap: 1` and configure `PointCloudMapping.loadFilename`.
* Extraction of **ORB** keypoints via **CUDA**. 
  - This capability can be optionally activated by using the option `USE_CUDA` in [config.sh](./config.sh) 
* Different methods can be used with calibrated stereo cameras for estimating depth maps: *libelas*, *libsgm*, *opencv* (these methods may need more fine tuning).
  - Use the option `StereoDense.type` to select your preferred method in the yaml settings for stereo cameras. This will work with your stereo datasets when `PointCloudMapping.on` is set to 1.  
* Some parts of the original ORBSLAM code were improved or optimized.
* A **new version of g2o** is supported (*tags/20230223_git*). This can be enabled by setting the option `WITH_G2O_NEW` to `ON` in the main `CMakeLists.txt` of PLVS. Note that the new version of g2o will be automatically installed for you by the main build script (`build.sh` → `build_thirdparty.sh` → `install_local_g2o_new.sh`).
* **Smart pointers** to manage points and lines (WIP for keyframes). See the file [Pointers.h](include/Pointers.h).
* **MapOjbect**: Experimental representation for planar objects (WIP for PLVS II).
* C++17 support. This can be configured at global level in [config.sh](./config.sh) by setting the variable `CPP_STANDARD_VERSION`.
* Many convenient scripts are provided for launching apps, benchmarking and monitoring the system. See the `Scripts` and the `Benchmarking` folders.
  
**Note**: PLVS is an active project. The main *README* is under construction and will be updated soon with further information and details. Code improvements are coming soon. 


You can find further details and videos on this [page](https://www.luigifreda.com/research/plvs-an-open-source-rgb-d-and-stereo-slam-for-volumetric-reconstruction-and-3d-incremental-segmentation/) and in the following **document**:          
**[PLVS: A SLAM System with Points, Lines, Volumetric Mapping, and 3D Incremental Segmentation](https://arxiv.org/pdf/2309.10896.pdf)**         
*Luigi Freda* 
