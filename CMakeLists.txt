cmake_minimum_required(VERSION 2.8)
project(plvs)

#====================================
# options

set(CPP_STANDARD_VERSION "17" CACHE STRING "Desired C++ standard version") # we need c++17 since nvcc does not support c++20 (probably we can try mixing standards and let nvcc use c++17 ... not sure about the result)
set(OPENCV_VERSION "4" CACHE STRING "Desired OpenCV version") # this is the first target version that is searched for 

set(BUILD_WITH_MARCH_NATIVE ON  CACHE BOOL "Build with \"-march native\"")

set(WITH_FASTFUSION     ON  CACHE BOOL "Add fastfusion support") # for enabling RGBD dense reconstruction via fastfusion (it needs Intel SIMD)
set(WITH_G2O_NEW        OFF CACHE BOOL "Add g2o new support") # for using the new version of g2o with Cholmod and Csparse solvers (and more...)
set(WITH_LIBELAS        ON  CACHE BOOL "Add libelas support")  # for stereo dense reconstruction (it needs Intel SIMD, optionally can be compiled with CUDA)
set(WITH_LIBSGM         ON  CACHE BOOL "Add stereo libsgm support")  # for stereo dense reconstruction (it needs CUDA)
set(WITH_OPENMP         ON  CACHE BOOL "Add OpenMP support") # useful for PCL (since we use the macro PCL_NO_PRECOMPILE!)
set(WITH_CUDA           OFF CACHE BOOL "Add CUDA support")
set(WITH_TRACY_PROFILER OFF CACHE BOOL "Add Tracy profiler support.")
set(WITH_RERUN          OFF CACHE BOOL "Add Rerun support.")
set(WITH_LOCAL_PCL      OFF  CACHE BOOL "Add local PCL support")

#====================================
# c++ standard 

# Set the C++ standard 
set(CMAKE_CXX_STANDARD ${CPP_STANDARD_VERSION})
set(CMAKE_CXX_STANDARD_REQUIRED ON)

#====================================
# general cmake options 

# NOTE: this is to remove the tons of cmake warning coming from importing ext packages 
# see here: https://github.com/PointCloudLibrary/pcl/issues/3680
# when this is fixed, we can remove the following 3 lines.
if(NOT DEFINED CMAKE_SUPPRESS_DEVELOPER_WARNINGS)
     set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS 1 CACHE INTERNAL "No dev warnings")
endif()

# This option allows the generations of a file compile_commands.json in our build folder: that file contains the full command line to compile individual source files
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#====================================
# compiler and build options 

set(LIBRARY_TYPE SHARED)
#set(LIBRARY_TYPE STATIC)  

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

#set(CMAKE_BUILD_TYPE Debug)  # force Debug 

message("Build type: " ${CMAKE_BUILD_TYPE})

#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall   -O3")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3")
#set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -march=native")
#set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -march=native")

if(CMAKE_BUILD_TYPE STREQUAL "Release")

    # Best release
    set(MY_FLAGS "-Wall -O3 -fPIC -DNDEBUG -Wno-unused-parameter -Wno-unused-function") # -Wextra -Wno-deprecated

    if(BUILD_WITH_MARCH_NATIVE)
        set(MY_FLAGS "${MY_FLAGS} -march=native")      
    endif()

    #set(MY_FLAGS "-Wall -Wextra -O3 -pthread -ftree-vectorize -funroll-loops")

    # Best release Nvidia Jetson TX2
    #set(MY_FLAGS "-Wall -O3 -ftree-vectorize -funroll-loops -pthread -Wextra")

    if(WITH_TX2)
        # Best release Nvidia Jetson TX2
        #set(MY_FLAGS "-Wall -O3 -march=armv8-a+crypto -mcpu=cortex-a57+crypto -Wextra -ffast-math -flto -fPIC -DNDEBUG")
    endif()
else()

    message(AUTHOR_WARNING "\n!!!Compiling in Debug mode!!!\n")
    # Debug
    #set(MY_FLAGS "-Wall -O3  -g -pthread")

    # Debug with g2o compiled with -march=native; this should avoid g2o crashes (see "Mismatched build flags" on https://github.com/ethz-asl/eigen_catkin/wiki/Eigen-Memory-Issues#memory-misalignment )
    set(MY_FLAGS "-Wall  -g -O3 -fPIC -pthread") # -fsanitize=address  

    if(BUILD_WITH_MARCH_NATIVE)
        set(MY_FLAGS "${MY_FLAGS} -march=native")      
    endif()

    # this makes g2o crash if g2o is compiled with optimizations 
    #set(MY_FLAGS "-Wall  -g")
endif()

# Add OpenMP flags
if( WITH_OPENMP )
    set(MY_FLAGS "${MY_FLAGS} -fopenmp")   
endif()

# get and store all the active flags 
set(MY_C_FLAGS "${CMAKE_C_FLAGS} ${MY_FLAGS}")
set(MY_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${MY_FLAGS}")    

# apply flags 
set(CMAKE_C_FLAGS "${MY_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${MY_CXX_FLAGS}")

message(STATUS "CMAKE_CXX_FLAGS: ${CMAKE_CXX_FLAGS}")

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

add_definitions(-DCOMPILEDWITHC11)  #TODO: replace the macro everywhere with COMPILEDWITHC14 or completely remove it

set(SHADERS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/Shaders" CACHE PATH "Where the shaders live")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DSHADERS_DIR=${SHADERS_DIR}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DSHADERS_DIR=${SHADERS_DIR}")

#====================================
# cmake modules

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

#====================================
# CUDA 
message(STATUS "WITH_CUDA: " ${WITH_CUDA})

if (WITH_CUDA)
    find_package(CUDA REQUIRED)

    set(CUDA_PROPAGATE_HOST_FLAGS OFF)
    SET(CUDA_HOST_COMPILER /usr/bin/g++)

    include_directories(${CUDA_INCLUDE_DIRS})

    # Set files to build
    set(CUDA_SOURCES
        src/cuda/Allocator_gpu.cu
        src/cuda/Fast_gpu.cu
        src/cuda/Orb_gpu.cu
        src/cuda/Cuda.cu
    )

    set(CUDA_ARCH_BIN "35 50 52 53 61 62 75 80 86" CACHE STRING "Specify 'real' GPU arch to build binaries for, BIN(PTX) format is supported. Example: 1.3 2.1(1.3) or 13 21(13)")
    set(CUDA_ARCH_PTX "" CACHE STRING "Specify 'virtual' PTX arch to build PTX intermediate code for. Example: 1.0 1.2 or 10 12")

    include(${CMAKE_MODULE_PATH}/CudaComputeTargetFlags.cmake)
    APPEND_TARGET_ARCH_FLAGS()

    set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} "-std=c++${CMAKE_CXX_STANDARD} -use_fast_math")
    set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} "-Xcompiler;-fPIC;")
    #set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} "--ftz=true;--prec-div=false;--prec-sqrt=false")

    if(CMAKE_BUILD_TYPE STREQUAL "Release")
        set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} "-O3 ")
    else()
        set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} "-g ")        
    endif()

    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DUSE_CUDA")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DUSE_CUDA")
endif()

#====================================
# packages 

find_package(Eigen3 3.1.0 REQUIRED)
message(STATUS "eigen include: ${EIGEN3_INCLUDE_DIR}")


find_package(Boost REQUIRED COMPONENTS thread system serialization)


# Check fastfusion support and SIMD supports
include(${CMAKE_MODULE_PATH}/CheckFastfusionSupport.cmake)
if(BUILD_FASTFUSION AND WITH_FASTFUSION)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DUSE_FASTFUSION")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DUSE_FASTFUSION")
    message(STATUS "Building fastfusion! ***************")
else()
    message(STATUS "fastfusion disabled! ***************")
endif()

if(NOT HAVE_SSE3)
    set(WITH_LIBELAS OFF)
    message(STATUS "libelas disabled! ***************")
endif()


message(STATUS "Desired OpenCV version: ${OPENCV_VERSION}")
find_package(OpenCV ${OPENCV_VERSION} REQUIRED)
message(STATUS "found OpenCV version: ${OpenCV_VERSION}")
message(STATUS "opencv include: ${OpenCV_INCLUDE_DIRS}")
message(STATUS "opencv lib dirs: ${OpenCV_INSTALL_PATH}")
message(STATUS "opencv libs: ${OpenCV_LIBS}")


set(Pangolin_DIR ${PROJECT_SOURCE_DIR}/Thirdparty/Pangolin/build/) # this seems to work in general 
#set(CMAKE_PREFIX_PATH ${PROJECT_SOURCE_DIR}/Thirdparty/Pangolin/install/lib/cmake/Pangolin/) # this does not work (after make install) if we set the RPATH!
#set(Pangolin_DIR ${PROJECT_SOURCE_DIR}/Thirdparty/Pangolin/install/) # this does not work (after make install) if we set the RPATH!
message(STATUS "set custom pangolin folder: ${Pangolin_DIR}")
find_package(Pangolin REQUIRED)
message(STATUS "pangolin include: ${Pangolin_INCLUDE_DIRS}")
message(STATUS "pangolin libs: ${Pangolin_LIBRARY}")


find_package(realsense2)


if(WITH_LOCAL_PCL)
    set(CMAKE_PREFIX_PATH ${PROJECT_SOURCE_DIR}/Thirdparty/pcl/install/share/pcl)
endif()
find_package(PCL 1.7 REQUIRED)
message(STATUS "pcl include: ${PCL_INCLUDE_DIRS}")
message(STATUS "pcl libs: ${PCL_LIBRARY_DIRS}")
message(STATUS "pcl definitions: ${PCL_DEFINITIONS}")
add_definitions(-DPCL_NO_PRECOMPILE)  # this is strictly required since we defined a new pcl point type
if(WITH_CUDA)
    # N.B.: the lines of this block allow to compile CUDA with PCL without troubles
    get_directory_property(dir_defs DIRECTORY ${CMAKE_SOURCE_DIR} COMPILE_DEFINITIONS)
    set(vtk_flags)
    foreach(it ${dir_defs})
        if(it MATCHES "vtk*")
        list(APPEND vtk_flags ${it})
        endif()
    endforeach()
    foreach(d ${vtk_flags})
        remove_definitions(-D${d})
    endforeach()
endif()
add_definitions( ${PCL_DEFINITIONS} )
link_directories( ${PCL_LIBRARY_DIRS})


find_package(GLOG REQUIRED)
if(NOT GLOG_FOUND)
	message(FATAL_ERROR "please run: sudo apt-get install libgoogle-glog-dev " )
endif()


find_package(octomap REQUIRED)
message(STATUS "octomap include: ${OCTOMAP_INCLUDE_DIRS}")
message(STATUS "octomap lib dirs: ${OCTOMAP_LIBRARY_DIRS}")
message(STATUS "octomap libs: ${OCTOMAP_LIBRARIES}")


find_package(Protobuf REQUIRED)
message(STATUS "protobuf libs: ${PROTOBUF_LIBRARIES}")


find_package(LibCrypto REQUIRED)


find_package(PkgConfig REQUIRED)
pkg_check_modules(GLFW REQUIRED glfw3)


if(BUILD_FASTFUSION)
    set(FASTFUSION_INCLUDES 
    ${PROJECT_SOURCE_DIR}/Thirdparty/fastfusion/include
    )
    set(FASTFUSION_LIBRARIES
    ${PROJECT_SOURCE_DIR}/Thirdparty/fastfusion/lib/libauxiliary.a
    ${PROJECT_SOURCE_DIR}/Thirdparty/fastfusion/lib/libcamerautils.a
    ${PROJECT_SOURCE_DIR}/Thirdparty/fastfusion/lib/libgeometryfusion_aos.a
    ${PROJECT_SOURCE_DIR}/Thirdparty/fastfusion/lib/libgeometryfusion_mipmap_cpu.a
    )
endif()


if(WITH_LIBELAS)
    if(NOT CUDA_FOUND)
        find_package(CUDA) # make the check if not already done above (CUDA are required if libelas )
    endif()
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DUSE_LIBELAS")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DUSE_LIBELAS")
    if(CUDA_FOUND)
        message(STATUS "building with libelas CUDA ***************")
    else()
        message(STATUS "building with libelas CPU ***************")
    endif()
    set(LIBELAS_INCLUDES 
    ${PROJECT_SOURCE_DIR}/Thirdparty/libelas-gpu/include
    )
    set(LIBELAS_LIBRARIES 
    ${PROJECT_SOURCE_DIR}/Thirdparty/libelas-gpu/lib/liblibelas_gpu.a  # this contains or not CUDA 
    )
else()
    message(STATUS "libelas disabled! ***************")
endif()


if(WITH_LIBSGM)
    if(NOT CUDA_FOUND)
        find_package(CUDA) # make the check if not already done above
    endif()
    if(CUDA_FOUND)
        set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DUSE_LIBSGM")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DUSE_LIBSGM")
        message(STATUS "building with stereo libsgm CUDA ***************")
        set(LIBSGM_INCLUDES 
        ${PROJECT_SOURCE_DIR}/Thirdparty/libsgm/include
        )
        set(LIBSGM_LIBRARIES 
        ${PROJECT_SOURCE_DIR}/Thirdparty/libsgm/lib/libsgm.a  # this contains CUDA 
        )
    else()
        message(STATUS "stereo libsgm disabled! ***************")
    endif()
else()
    message(STATUS "stereo libsgm disabled! ***************")
endif()


if( WITH_G2O_NEW )
    # In order to set RPATH (instead of RUNPATH) for g2o and other libraries linking issue.
    # ISSUE: g2o binaries link to system g2o (here /opt/ros/noetic/lib/libg2o_core.so) and crash (different version); 
    # we need to set LD_LIBRARY_PATH for correct linking
    # https://stackoverflow.com/questions/47117443/dynamic-linking-with-rpath-not-working-under-ubuntu-17-10 
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wl,--disable-new-dtags")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wl,--disable-new-dtags")          
    find_package(Cholmod)
    include_directories(${CHOLMOD_INCLUDE_DIR})        
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DUSE_G2O_NEW")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DUSE_G2O_NEW")    
    # NOTE: use the following block of lines to test latest g2o version support.
    #       If it crashes. Check this link: https://github.com/rst-tu-dortmund/teb_local_planner/issues/336
    set(CMAKE_PREFIX_PATH ${PROJECT_SOURCE_DIR}/Thirdparty/g2o_new/install/lib/cmake/g2o/)
    set(G2O_ROOT ${PROJECT_SOURCE_DIR}/Thirdparty/g2o_new/install/)
    message(STATUS "set custom g2o root folder: ${G2O_ROOT}")
    find_package(G2O REQUIRED)
    # used in the latest g2o version 
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DG2O_USE_VENDORED_CERES")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DG2O_USE_VENDORED_CERES")
else()
    # g2o provided with ORBSLAM3
    set(G2O_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/)
    set(G2O_LIBS ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so)
endif()
message(STATUS "g2o include: ${G2O_INCLUDE_DIR}")
message(STATUS "g2o lib dirs: ${G2O_LIB_DIR}")
message(STATUS "g2o libs: ${G2O_LIBS}")

if(WITH_TRACY_PROFILER)
  if(EXISTS "/usr/local/lib/libTracyClient.a")
    message(STATUS "Found Tracy")
    add_definitions(-DTRACY_ENABLE)
    set(TRACY_LIBS /usr/local/lib/libTracyClient.a -ldl -lpthread)
  endif()
endif()

if(WITH_RERUN)
    # rerun_sdk
    add_definitions(-DRERUN_ENABLED) 
    set(CMAKE_PREFIX_PATH ${PROJECT_SOURCE_DIR}/Thirdparty/rerun/install/lib/cmake/rerun_sdk/)
    set(Rerun_DIR ${PROJECT_SOURCE_DIR}/Thirdparty/rerun/install/)
    message(STATUS "set custom rerun folder: ${Rerun_DIR}")
    find_package(rerun_sdk REQUIRED)
    message(STATUS "rerun include: ${RERUN_LIBRARIES}")
endif()

#====================================
# includes 

include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/include
${PROJECT_SOURCE_DIR}/include/CameraModels
${G2O_INCLUDE_DIR}
${PROJECT_SOURCE_DIR}/Thirdparty/volumetric_mapping/include
${PROJECT_SOURCE_DIR}/Thirdparty/open_chisel/include
${PROJECT_SOURCE_DIR}/Thirdparty/chisel_server/include
${FASTFUSION_INCLUDES}
${PROJECT_SOURCE_DIR}/Thirdparty/voxblox/include
${PROJECT_SOURCE_DIR}/Thirdparty/voxblox_server/include
${PROJECT_SOURCE_DIR}/Thirdparty/line_descriptor/include
${LIBELAS_INCLUDES}
${LIBSGM_INCLUDES}
${PROJECT_SOURCE_DIR}/Thirdparty/Sophus
${EIGEN3_INCLUDE_DIR}
${Pangolin_INCLUDE_DIRS}
${PCL_INCLUDE_DIRS}
${GLOG_INCLUDE_DIRS}
${OCTOMAP_INCLUDE_DIRS}
${OpenCV_INCLUDE_DIRS}
${Boost_INCLUDE_DIRS}
${GLFW_INCLUDE_DIRS}
${CHOLMOD_INCLUDE_DIR}
)

# If RealSense SDK is found the library is added and its examples compiled
if(realsense2_FOUND)
    include_directories(${PROJECT_NAME} ${realsense_INCLUDE_DIR})
endif()

#====================================
# libs 

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)


set(EXTERNAL_CORE_LIBS   # core external libs 
${OpenCV_LIBS}
${EIGEN3_LIBS}
${Boost_LIBRARIES} boost_serialization boost_thread # FIXME: remove this and let cmake package finder get the info!
#Boost::boost # NOTE this does not work!
${Pangolin_LIBRARIES}
${G2O_LIBS}
${PCL_LIBRARIES}
${GLOG_LIBRARIES}
${OCTOMAP_LIBRARIES}
${PROTOBUF_LIBRARIES}
${LibCrypto_LIBRARY}
${GLFW_LIBRARIES}
${CHOLMOD_LIBRARIES}
)
if(CUDA_FOUND)
    set(EXTERNAL_CORE_LIBS ${EXTERNAL_CORE_LIBS} ${CUDA_LIBRARIES}) # ${CUDA_LIBRARIES} are used just for stereo disparity computation when libelas or libsgm support are added 
endif()
if(WITH_RERUN)
    set(EXTERNAL_CORE_LIBS ${EXTERNAL_CORE_LIBS} rerun_sdk)
endif()

# If RealSense SDK is found the library is added and its examples compiled
if(realsense2_FOUND)
    message(STATUS "found REALSENSE LIB: ${realsense2_LIBRARY}")
    #target_link_libraries(${PROJECT_NAME} ${realsense2_LIBRARY})
    set(EXTERNAL_CORE_LIBS ${EXTERNAL_CORE_LIBS} ${realsense2_LIBRARY})
endif()


set(EXTERNAL_LIBS  # Thirdparty libs 
${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
${PROJECT_SOURCE_DIR}/Thirdparty/line_descriptor/lib/liblinedesc.a
${PROJECT_SOURCE_DIR}/Thirdparty/volumetric_mapping/lib/libvolumetric_mapping.so
${PROJECT_SOURCE_DIR}/Thirdparty/chisel_server/lib/libchisel_server.a
${PROJECT_SOURCE_DIR}/Thirdparty/open_chisel/lib/libopen_chisel.a
${FASTFUSION_LIBRARIES}
${PROJECT_SOURCE_DIR}/Thirdparty/voxblox/lib/libvoxblox.a
${PROJECT_SOURCE_DIR}/Thirdparty/voxblox/lib/libvoxblox_proto.a
${PROJECT_SOURCE_DIR}/Thirdparty/voxblox_server/lib/libvoxblox_server.a
${LIBELAS_LIBRARIES}
${LIBSGM_LIBRARIES}
)

link_directories( 
  ${G2O_LIB_DIR}  
  ${PCL_LIBRARY_DIRS}
)

#====================================
# source files and folders to build 

#add_library(${PROJECT_NAME} SHARED
set(CPU_SOURCES 
src/System.cc
src/Tracking.cc
src/LocalMapping.cc
src/LoopClosing.cc
src/ORBextractor.cc
src/ORBmatcher.cc
src/FrameDrawer.cc
src/Converter.cc
src/MapPoint.cc
src/KeyFrame.cc
src/Atlas.cc
src/Map.cc
src/MapDrawer.cc
src/Optimizer.cc
src/Frame.cc
src/KeyFrameDatabase.cc
src/Sim3Solver.cc
src/Viewer.cc
src/ImuTypes.cc
src/G2oTypes.cc
src/G2oLineTypes.cc
src/CameraModels/Pinhole.cpp
src/CameraModels/KannalaBrandt8.cpp
src/OptimizableTypes.cpp
src/OptimizableLineTypes.cpp
src/MLPnPsolver.cpp
src/GeometricTools.cc
src/TwoViewReconstruction.cc
src/Config.cc
src/Settings.cc
include/System.h
include/Tracking.h
include/LocalMapping.h
include/LoopClosing.h
include/ORBextractor.h
include/ORBmatcher.h
include/FrameDrawer.h
include/Converter.h
include/MapPoint.h
include/KeyFrame.h
include/Atlas.h
include/Map.h
include/MapDrawer.h
include/Optimizer.h
include/Frame.h
include/KeyFrameDatabase.h
include/Sim3Solver.h
include/Viewer.h
include/ImuTypes.h
include/G2oTypes.h
include/G2oLineTypes.h
include/CameraModels/GeometricCamera.h
include/CameraModels/Pinhole.h
include/CameraModels/KannalaBrandt8.h
include/OptimizableTypes.h
include/OptimizableLineTypes.h
include/MLPnPsolver.h
include/GeometricTools.h
include/TwoViewReconstruction.h
include/SerializationUtils.h
include/Config.h
include/Settings.h
###
src/MapLine.cc
src/LineExtractor.cc
src/LineMatcher.cc
src/MapObject.cc
src/Pointers.cc
###
src/PointCloudMapping.cc
src/PointCloudKeyFrame.cc
src/PointCloudDrawer.cc
src/OctomapManager.cc
src/ColorOctomapServer.cc
src/OctreePointCloudCentroid.cc
src/LabelMap.cc
#src/FastFusionManager.cc
src/PointCloudMap.cc
src/PointCloudMapChisel.cc
src/PointCloudMapFastFusion.cc
src/PointCloudMapVoxblox.cc
src/PointCloudMapOctreePointCloud.cc
src/PointCloudMapOctomap.cc
src/PointCloudMapVoxelGridFilter.cc
src/StereoDisparity.cc
src/KeyFrameSearchTree.cc
src/PointCloudAtlas.cc
src/PointCloudMapTypes.cc
### 
src/GlObject.cc
src/GlPointCloud.cc
src/GlObjectList.cc
###
src/g2o/types_sba_line.cpp
src/g2o/types_six_dof_expmap2.cpp
src/g2o/types_seven_dof_expmap2.cpp
)

#add_subdirectory(Thirdparty/g2o)

#target_link_libraries(${PROJECT_NAME}
#${OpenCV_LIBS}
#${EIGEN3_LIBS}
#${Pangolin_LIBRARIES}
#${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
#${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
#-lboost_serialization
#-lcrypto
#)

if (WITH_CUDA)
    cuda_compile(CUDA_OBJS ${CUDA_SOURCES}) 
endif()

add_library(${PROJECT_NAME} ${LIBRARY_TYPE}
    ${CUDA_OBJS}
    ${CPU_SOURCES}
)

target_link_libraries(${PROJECT_NAME}
${EXTERNAL_LIBS} ${EXTERNAL_CORE_LIBS})


set(CORE_LIBS ${PROJECT_NAME})

message(STATUS "Boost_LIBRARIES: ${Boost_LIBRARIES}")
message(STATUS "EXTERNAL_CORE_LIBS: ${EXTERNAL_CORE_LIBS}")


### Build examples


# RGB-D examples
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/RGB-D)

add_executable(rgbd_tum
        Examples/RGB-D/rgbd_tum.cc)
target_link_libraries(rgbd_tum ${CORE_LIBS} ${EXTERNAL_LIBS} ${EXTERNAL_CORE_LIBS})

if(realsense2_FOUND)
    add_executable(rgbd_realsense_D435i
            Examples/RGB-D/rgbd_realsense_D435i.cc)
    target_link_libraries(rgbd_realsense_D435i ${CORE_LIBS} ${EXTERNAL_LIBS} ${EXTERNAL_CORE_LIBS})
endif()


# RGB-D inertial examples
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/RGB-D-Inertial)

if(realsense2_FOUND)
    add_executable(rgbd_inertial_realsense_D435i
            Examples/RGB-D-Inertial/rgbd_inertial_realsense_D435i.cc)
    target_link_libraries(rgbd_inertial_realsense_D435i ${CORE_LIBS} ${EXTERNAL_LIBS} ${EXTERNAL_CORE_LIBS})
endif()

# Stereo examples
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Stereo)

add_executable(stereo_kitti
        Examples/Stereo/stereo_kitti.cc)
target_link_libraries(stereo_kitti ${CORE_LIBS} ${EXTERNAL_LIBS} ${EXTERNAL_CORE_LIBS})

add_executable(stereo_euroc
        Examples/Stereo/stereo_euroc.cc)
target_link_libraries(stereo_euroc ${CORE_LIBS} ${EXTERNAL_LIBS} ${EXTERNAL_CORE_LIBS})

add_executable(stereo_tum_vi
        Examples/Stereo/stereo_tum_vi.cc)
target_link_libraries(stereo_tum_vi ${CORE_LIBS} ${EXTERNAL_LIBS} ${EXTERNAL_CORE_LIBS})

if(realsense2_FOUND)
    add_executable(stereo_realsense_t265
            Examples/Stereo/stereo_realsense_t265.cc)
    target_link_libraries(stereo_realsense_t265 ${CORE_LIBS} ${EXTERNAL_LIBS} ${EXTERNAL_CORE_LIBS})

    add_executable(stereo_realsense_D435i
            Examples/Stereo/stereo_realsense_D435i.cc)
    target_link_libraries(stereo_realsense_D435i ${CORE_LIBS} ${EXTERNAL_LIBS} ${EXTERNAL_CORE_LIBS})
endif()

# Monocular examples
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Monocular)

add_executable(mono_tum
        Examples/Monocular/mono_tum.cc)
target_link_libraries(mono_tum ${CORE_LIBS} ${EXTERNAL_LIBS} ${EXTERNAL_CORE_LIBS})

add_executable(mono_kitti
        Examples/Monocular/mono_kitti.cc)
target_link_libraries(mono_kitti ${CORE_LIBS} ${EXTERNAL_LIBS} ${EXTERNAL_CORE_LIBS})

add_executable(mono_euroc
        Examples/Monocular/mono_euroc.cc)
target_link_libraries(mono_euroc ${CORE_LIBS} ${EXTERNAL_LIBS} ${EXTERNAL_CORE_LIBS})

add_executable(mono_tum_vi
        Examples/Monocular/mono_tum_vi.cc)
target_link_libraries(mono_tum_vi ${CORE_LIBS} ${EXTERNAL_LIBS} ${EXTERNAL_CORE_LIBS})

if(realsense2_FOUND)
    add_executable(mono_realsense_t265
            Examples/Monocular/mono_realsense_t265.cc)
    target_link_libraries(mono_realsense_t265 ${CORE_LIBS} ${EXTERNAL_LIBS} ${EXTERNAL_CORE_LIBS})

    add_executable(mono_realsense_D435i
            Examples/Monocular/mono_realsense_D435i.cc)
    target_link_libraries(mono_realsense_D435i ${CORE_LIBS} ${EXTERNAL_LIBS} ${EXTERNAL_CORE_LIBS})
endif()

# Monocular inertial examples
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Monocular-Inertial)

add_executable(mono_inertial_euroc
        Examples/Monocular-Inertial/mono_inertial_euroc.cc)
target_link_libraries(mono_inertial_euroc ${CORE_LIBS} ${EXTERNAL_LIBS} ${EXTERNAL_CORE_LIBS})

add_executable(mono_inertial_tum_vi
        Examples/Monocular-Inertial/mono_inertial_tum_vi.cc)
target_link_libraries(mono_inertial_tum_vi ${CORE_LIBS} ${EXTERNAL_LIBS} ${EXTERNAL_CORE_LIBS})

if(realsense2_FOUND)
    add_executable(mono_inertial_realsense_t265
            Examples/Monocular-Inertial/mono_inertial_realsense_t265.cc)
    target_link_libraries(mono_inertial_realsense_t265 ${CORE_LIBS} ${EXTERNAL_LIBS} ${EXTERNAL_CORE_LIBS})

    add_executable(mono_inertial_realsense_D435i
            Examples/Monocular-Inertial/mono_inertial_realsense_D435i.cc)
    target_link_libraries(mono_inertial_realsense_D435i ${CORE_LIBS} ${EXTERNAL_LIBS} ${EXTERNAL_CORE_LIBS})
endif()

# Stereo Inertial examples
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Stereo-Inertial)

add_executable(stereo_inertial_euroc
        Examples/Stereo-Inertial/stereo_inertial_euroc.cc)
target_link_libraries(stereo_inertial_euroc ${CORE_LIBS} ${EXTERNAL_LIBS} ${EXTERNAL_CORE_LIBS})

add_executable(stereo_inertial_tum_vi
        Examples/Stereo-Inertial/stereo_inertial_tum_vi.cc)
target_link_libraries(stereo_inertial_tum_vi ${CORE_LIBS} ${EXTERNAL_LIBS} ${EXTERNAL_CORE_LIBS})

if(realsense2_FOUND)
    add_executable(stereo_inertial_realsense_t265
            Examples/Stereo-Inertial/stereo_inertial_realsense_t265.cc)
    target_link_libraries(stereo_inertial_realsense_t265 ${CORE_LIBS} ${EXTERNAL_LIBS} ${EXTERNAL_CORE_LIBS})

    add_executable(stereo_inertial_realsense_D435i
            Examples/Stereo-Inertial/stereo_inertial_realsense_D435i.cc)
    target_link_libraries(stereo_inertial_realsense_D435i ${CORE_LIBS} ${EXTERNAL_LIBS} ${EXTERNAL_CORE_LIBS})
endif()

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Calibration)
if(realsense2_FOUND)
    add_executable(recorder_realsense_D435i
            Examples/Calibration/recorder_realsense_D435i.cc)
    target_link_libraries(recorder_realsense_D435i ${CORE_LIBS} ${EXTERNAL_LIBS} ${EXTERNAL_CORE_LIBS})

    add_executable(recorder_realsense_T265
            Examples/Calibration/recorder_realsense_T265.cc)
    target_link_libraries(recorder_realsense_T265 ${CORE_LIBS} ${EXTERNAL_LIBS} ${EXTERNAL_CORE_LIBS})
endif()


###  Old examples

# RGB-D examples
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples_old/RGB-D)

add_executable(rgbd_tum_old
        Examples_old/RGB-D/rgbd_tum.cc)
target_link_libraries(rgbd_tum_old ${CORE_LIBS} ${EXTERNAL_LIBS} ${EXTERNAL_CORE_LIBS})


add_executable(rgbd_tum_ar 
        Examples_old/RGB-D/rgbd_tum_ar.cc Examples_old/RGB-D/ViewerAR.h Examples_old/RGB-D/ViewerAR.cc)
target_link_libraries(rgbd_tum_ar ${CORE_LIBS} ${EXTERNAL_LIBS} ${EXTERNAL_CORE_LIBS})

add_executable(rgbd_inseg
	Examples_old/RGB-D/rgbd_inseg.cc)
target_link_libraries(rgbd_inseg ${CORE_LIBS} ${EXTERNAL_LIBS} ${EXTERNAL_CORE_LIBS})

if(realsense2_FOUND)
    add_executable(rgbd_realsense_D435i_old
            Examples_old/RGB-D/rgbd_realsense_D435i.cc)
    target_link_libraries(rgbd_realsense_D435i_old ${CORE_LIBS} ${EXTERNAL_LIBS} ${EXTERNAL_CORE_LIBS})
endif()


# RGB-D inertial examples
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples_old/RGB-D-Inertial)

if(realsense2_FOUND)
    add_executable(rgbd_inertial_realsense_D435i_old
            Examples_old/RGB-D-Inertial/rgbd_inertial_realsense_D435i.cc)
    target_link_libraries(rgbd_inertial_realsense_D435i_old ${CORE_LIBS} ${EXTERNAL_LIBS} ${EXTERNAL_CORE_LIBS})
endif()

# Stereo examples
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples_old/Stereo)

add_executable(stereo_kitti_old
        Examples_old/Stereo/stereo_kitti.cc)
target_link_libraries(stereo_kitti_old ${CORE_LIBS} ${EXTERNAL_LIBS} ${EXTERNAL_CORE_LIBS})

add_executable(stereo_euroc_old
        Examples_old/Stereo/stereo_euroc.cc)
target_link_libraries(stereo_euroc_old ${CORE_LIBS} ${EXTERNAL_LIBS} ${EXTERNAL_CORE_LIBS})

add_executable(stereo_tum_vi_old
        Examples_old/Stereo/stereo_tum_vi.cc)
target_link_libraries(stereo_tum_vi_old ${CORE_LIBS} ${EXTERNAL_LIBS} ${EXTERNAL_CORE_LIBS})

if(realsense2_FOUND)
    add_executable(stereo_realsense_t265_old
            Examples_old/Stereo/stereo_realsense_t265.cc)
    target_link_libraries(stereo_realsense_t265_old ${CORE_LIBS} ${EXTERNAL_LIBS} ${EXTERNAL_CORE_LIBS})

    add_executable(stereo_realsense_D435i_old
            Examples_old/Stereo/stereo_realsense_D435i.cc)
    target_link_libraries(stereo_realsense_D435i_old ${CORE_LIBS} ${EXTERNAL_LIBS} ${EXTERNAL_CORE_LIBS})
endif()

# Monocular examples
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples_old/Monocular)

add_executable(mono_tum_old
        Examples_old/Monocular/mono_tum.cc)
target_link_libraries(mono_tum_old ${CORE_LIBS} ${EXTERNAL_LIBS} ${EXTERNAL_CORE_LIBS})

add_executable(mono_kitti_old
        Examples_old/Monocular/mono_kitti.cc)
target_link_libraries(mono_kitti_old ${CORE_LIBS} ${EXTERNAL_LIBS} ${EXTERNAL_CORE_LIBS})

add_executable(mono_euroc_old
        Examples_old/Monocular/mono_euroc.cc)
target_link_libraries(mono_euroc_old ${CORE_LIBS} ${EXTERNAL_LIBS} ${EXTERNAL_CORE_LIBS})

add_executable(mono_tum_vi_old
        Examples_old/Monocular/mono_tum_vi.cc)
target_link_libraries(mono_tum_vi_old ${CORE_LIBS} ${EXTERNAL_LIBS} ${EXTERNAL_CORE_LIBS})

if(realsense2_FOUND)
    add_executable(mono_realsense_t265_old
            Examples_old/Monocular/mono_realsense_t265.cc)
    target_link_libraries(mono_realsense_t265_old ${CORE_LIBS} ${EXTERNAL_LIBS} ${EXTERNAL_CORE_LIBS})

    add_executable(mono_realsense_D435i_old
            Examples_old/Monocular/mono_realsense_D435i.cc)
    target_link_libraries(mono_realsense_D435i_old ${CORE_LIBS} ${EXTERNAL_LIBS} ${EXTERNAL_CORE_LIBS})
endif()

# Monocular inertial examples
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples_old/Monocular-Inertial)

add_executable(mono_inertial_euroc_old
        Examples_old/Monocular-Inertial/mono_inertial_euroc.cc)
target_link_libraries(mono_inertial_euroc_old ${CORE_LIBS} ${EXTERNAL_LIBS} ${EXTERNAL_CORE_LIBS})

add_executable(mono_inertial_tum_vi_old
        Examples_old/Monocular-Inertial/mono_inertial_tum_vi.cc)
target_link_libraries(mono_inertial_tum_vi_old ${CORE_LIBS} ${EXTERNAL_LIBS} ${EXTERNAL_CORE_LIBS})

if(realsense2_FOUND)
    add_executable(mono_inertial_realsense_t265_old
            Examples_old/Monocular-Inertial/mono_inertial_realsense_t265.cc)
    target_link_libraries(mono_inertial_realsense_t265_old ${CORE_LIBS} ${EXTERNAL_LIBS} ${EXTERNAL_CORE_LIBS})

    add_executable(mono_inertial_realsense_D435i_old
            Examples_old/Monocular-Inertial/mono_inertial_realsense_D435i.cc)
    target_link_libraries(mono_inertial_realsense_D435i_old ${CORE_LIBS} ${EXTERNAL_LIBS} ${EXTERNAL_CORE_LIBS})
endif()

# Stereo Inertial examples
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples_old/Stereo-Inertial)

add_executable(stereo_inertial_euroc_old
        Examples_old/Stereo-Inertial/stereo_inertial_euroc.cc)
target_link_libraries(stereo_inertial_euroc_old ${CORE_LIBS} ${EXTERNAL_LIBS} ${EXTERNAL_CORE_LIBS})

add_executable(stereo_inertial_tum_vi_old
        Examples_old/Stereo-Inertial/stereo_inertial_tum_vi.cc)
target_link_libraries(stereo_inertial_tum_vi_old ${CORE_LIBS} ${EXTERNAL_LIBS} ${EXTERNAL_CORE_LIBS})

if(realsense2_FOUND)
    add_executable(stereo_inertial_realsense_t265_old
            Examples_old/Stereo-Inertial/stereo_inertial_realsense_t265.cc)
    target_link_libraries(stereo_inertial_realsense_t265_old ${CORE_LIBS} ${EXTERNAL_LIBS} ${EXTERNAL_CORE_LIBS})

    add_executable(stereo_inertial_realsense_D435i_old
            Examples_old/Stereo-Inertial/stereo_inertial_realsense_D435i.cc)
    target_link_libraries(stereo_inertial_realsense_D435i_old ${CORE_LIBS} ${EXTERNAL_LIBS} ${EXTERNAL_CORE_LIBS})
endif()


set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Vocabulary)
add_executable(bin_vocabulary Vocabulary/bin_vocabulary.cpp)
target_link_libraries(bin_vocabulary ${CORE_LIBS} ${EXTERNAL_LIBS} ${EXTERNAL_CORE_LIBS})

if(EXISTS ${PROJECT_SOURCE_DIR}/test)
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
    #add_subdirectory(${PROJECT_SOURCE_DIR}/test) # uncomment to build tests/examples
endif()

message(STATUS "CMAKE_CXX_FLAGS: ${CMAKE_CXX_FLAGS}")
message(STATUS "CUDA_NVCC_FLAGS: ${CUDA_NVCC_FLAGS}")

