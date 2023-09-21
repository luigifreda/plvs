# PLVS II


PLVS is a real-time system that leverages sparse SLAM, volumetric mapping, and 3D unsupervised incremental segmentation. PLVS stands for **Points**, **Lines**, **Volumetric mapping**, and **Segmentation**. 

<p align="center">
<img src="Images/PLVS-lab.png"
alt="PLVS lab" max-width="585" border="1"/> 
<img src="Images/PLVS-Points-Lines-Vol-Seg.png"
alt="PLVS details" max-width="695" border="1"/> 
<img src="Images/PLVS-ar2.gif"
alt="PLVS augmented reality" height="180" border="1"/> 
<img src="Images/PLVS-details.gif"
alt="PLVS details" height="180" border="1"/> 
</p>

PLVS is available in two different versions.
- **PLVS I**: hosted in the branch `plvs1`. It is based on [ORB-SLAM2](https://github.com/raulmur/ORB_SLAM2), and supports mono, stereo, and RGB-D cameras.
- **PLVS II**: hosted in the `master` branch. It based on [ORB-SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3), and also supports camera systems provided with IMUs. 


You can find further details on this [page](https://www.luigifreda.com/research/plvs-an-open-source-rgb-d-and-stereo-slam-for-volumetric-reconstruction-and-3d-incremental-segmentation/) and in the following document:

**[PLVS: A SLAM System with Points, Lines, Volumetric Mapping, and 3D Incremental Segmentation](https://www.luigifreda.com/wp-content/uploads/2023/09/Freda-PLVS.pdf)**         
*Luigi Freda*        
CoRR 2023 



**Note**: PLVS is an active project. This *README* is under construction and will be updated soon with further information and details. 


  
  
## Quick start 

- Install basic dependencies:      
  `$ ./install_dependencies.sh`        
- Install opencv in a local folder:                 
  `$ ./install_local_opencv.sh`      
  (if you want, skip this step and set the variable `OpenCV_DIR` in `config.sh` with your local OpenCV path)     
- Build the framework:       
  `$ ./build.sh`


## Running the examples 

Once everything is built, you can enter in the `Scripts` folder and test the different examples. For instance you can configure and run: 
- `$ ./run_tum_rgbd.sh` for TUM RGB-D datasets 
- `$ ./run_kitti_stereo.sh` for KITTI datasets
- `$ ./run_euroc_stereo_inertial.sh` for Euroc datasets, stereo + inertial
- `$ ./run_euroc_stereo.sh` for Euroc datasets, only stereo
- `$ ./run_tum_vi_stereo.sh` for TUM VI datasets, only stereo
- `$ ./run_tum_vi_stereo_inertial.sh` for TUM VI datasets, stereo + inertial

In each of the above scripts, you have to configure the `DATASET_BASE_FOLDER`, the specific `DATASET` of interest, and the used `YAML` configuration file. In particular, each configuration file shows different sections with commented options.   

**Note**: PLVS is an active project. This *README* is under construction and will be updated soon with further information and details. 

## Credits  

* The *PLVS I* and *PLVS II* frameworks are based on [ORB-SLAM2](https://github.com/raulmur/ORB_SLAM2) and [ORB-SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3) respectively. Many thanks to their Authors for their great work and contributions to the open-source community. 

