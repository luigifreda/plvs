cmake_minimum_required(VERSION 3.5)
project(plvs)

# You should set the PYTHONPATH to your own python site-packages path
#set(ENV{PYTHONPATH} "/opt/ros/foxy/lib/python3.8/site-packages/")

set(CPP_STANDARD_VERSION "17" CACHE STRING "Desired C++ standard version") 
set(OPENCV_VERSION "4" CACHE STRING "Desired OpenCV version") # this is the first target version that is searched for 
set(BUILD_WITH_MARCH_NATIVE ON  CACHE BOOL "Build with \"-march native\"")
set(WITH_OPENMP             ON CACHE BOOL "Added OpenMP support") # useful for PCL (since we use the macro PCL_NO_PRECOMPILE!)
set(WITH_CUDA               OFF CACHE BOOL "Added CUDA support")
set(WITH_LOCAL_PCL          OFF  CACHE BOOL "Add local PCL support")


set(PLVS_ROOT_DIR ${PROJECT_SOURCE_DIR}/../../..)
message("PLVS_ROOT_DIR: ${PLVS_ROOT_DIR}")

#====================================
# c++ standard 

# Set the C++ standard 
set(CMAKE_CXX_STANDARD ${CPP_STANDARD_VERSION})
set(CMAKE_CXX_STANDARD_REQUIRED ON)

#====================================
# general cmake options 

# This option allows the generations of a file compile_commands.json in our build folder: that file contains the full command line to compile individual source files
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)


#====================================
# compiler and build options 

# if CMAKE_BUILD_TYPE is not specified, take 'Release' as default
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

#set(CMAKE_BUILD_TYPE Debug)  # force Debug 

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})


if(CMAKE_BUILD_TYPE STREQUAL "Release")

    # Best release
    set(MY_FLAGS "-Wall -Wextra -O3 -fPIC -DNDEBUG -Wno-unused-parameter -Wno-unused-function")

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

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#====================================

# cmake modules

LIST(APPEND CMAKE_MODULE_PATH ${PLVS_ROOT_DIR}/cmake_modules)

include(${CMAKE_MODULE_PATH}/CheckFastfusionSupport.cmake)
if(BUILD_FASTFUSION)
    message(STATUS "building fastfusion ***************")
else()
    message(STATUS "fastfusion disabled! ***************")
endif()



find_program(LSB_RELEASE lsb_release)
message("LSB_RELEASE: ${LSB_RELEASE}")

find_package(Eigen3 REQUIRED)

# set(Pangolin_DIR ${PLVS_ROOT_DIR}/Thirdparty/Pangolin/build/src)
set(Pangolin_DIR ${PLVS_ROOT_DIR}/Thirdparty/Pangolin/build/)
message(STATUS "set custom pangolin folder: ${Pangolin_DIR}")
find_package(Pangolin REQUIRED)
message(STATUS "pangolin include: ${Pangolin_INCLUDE_DIRS}")


if(WITH_LOCAL_PCL)
    set(CMAKE_PREFIX_PATH ${PLVS_ROOT_DIR}/Thirdparty/pcl/install/share/pcl)
endif()
find_package(PCL 1.7 REQUIRED )
message(STATUS "pcl include: ${PCL_INCLUDE_DIRS}")
message(STATUS "pcl libs: ${PCL_LIBRARY_DIRS}")
message(STATUS "pcl definitions: ${PCL_DEFINITIONS}")
#add_definitions( ${PCL_DEFINITIONS} )     # added below
#link_directories( ${PCL_LIBRARY_DIRS})    # added below

message(STATUS "Desired OpenCV version: ${OPENCV_VERSION}")
find_package(OpenCV ${OPENCV_VERSION} REQUIRED)
message(STATUS "found OpenCV version: ${OpenCV_VERSION}")
message(STATUS "opencv include: ${OpenCV_INCLUDE_DIRS}")
message(STATUS "opencv lib dirs: ${OpenCV_INSTALL_PATH}")
message(STATUS "opencv libs: ${OpenCV_LIBS}")

if(WITH_CUDA)
    find_package(CUDA REQUIRED)
endif()


find_package(GLOG REQUIRED)
if(NOT GLOG_FOUND)
	message(FATAL_ERROR "please run: sudo apt-get install libgoogle-glog-dev " )
endif()
find_package(Protobuf REQUIRED)
message(STATUS "protobuf libs: ${PROTOBUF_LIBRARIES}")


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
#find_package(Sophus REQUIRED)
#find_package(Pangolin REQUIRED)
#find_package(ORB_SLAM3 REQUIRED)


#====================================


include_directories(
${catkin_INCLUDE_DIRS}
${OpenCV_INCLUDE_DIRS}
${PCL_INCLUDE_DIRS}
${GLOG_INCLUDE_DIRS}
${CUDA_INCLUDE_DIRS}
)

include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/include
${PLVS_ROOT_DIR}
${PLVS_ROOT_DIR}/include
${PLVS_ROOT_DIR}/include/CameraModels
${PLVS_ROOT_DIR}/Thirdparty/line_descriptor/include
${PLVS_ROOT_DIR}/Thirdparty/Sophus
${Pangolin_INCLUDE_DIRS}
)

link_directories(${CUDA_LIBRARY_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})

add_definitions( ${PCL_DEFINITIONS} )
link_directories( ${PCL_LIBRARY_DIRS})

if(BUILD_FASTFUSION)
set(FASTFUSION_LIBS
    ${PLVS_ROOT_DIR}/Thirdparty/fastfusion/lib/libauxiliary.a
    ${PLVS_ROOT_DIR}/Thirdparty/fastfusion/lib/libcamerautils.a
    ${PLVS_ROOT_DIR}/Thirdparty/fastfusion/lib/libgeometryfusion_aos.a
    ${PLVS_ROOT_DIR}/Thirdparty/fastfusion/lib/libgeometryfusion_mipmap_cpu.a
    )
endif()


set(LIBS
${PLVS_ROOT_DIR}/lib/lib${PROJECT_NAME}.so
#${PLVS_ROOT_DIR}/lib/lib${PROJECT_NAME}_pc_mapping.so
${PLVS_ROOT_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
${PLVS_ROOT_DIR}/Thirdparty/g2o/lib/libg2o.so
${PLVS_ROOT_DIR}/Thirdparty/volumetric_mapping/lib/libvolumetric_mapping.so
${PLVS_ROOT_DIR}/Thirdparty/chisel_server/lib/libchisel_server.a
${PLVS_ROOT_DIR}/Thirdparty/open_chisel/lib/libopen_chisel.a
${PLVS_ROOT_DIR}/Thirdparty/voxblox_server/lib/libvoxblox_server.a
${PLVS_ROOT_DIR}/Thirdparty/voxblox/lib/libvoxblox.a
${PLVS_ROOT_DIR}/Thirdparty/voxblox/lib/libvoxblox_proto.a
${PLVS_ROOT_DIR}/Thirdparty/line_descriptor/lib/liblinedesc.a
${OpenCV_LIBS}
${EIGEN3_LIBS}
${Pangolin_LIBRARIES}
${FASTFUSION_LIBS}
${GLOG_LIBRARIES}
${PROTOBUF_LIBRARIES}
${CUDA_LIBRARIES}
${Boost_LIBRARIES}
)


add_executable(mono
  src/monocular/mono.cpp
  src/monocular/monocular-slam-node.cpp
)
ament_target_dependencies(mono rclcpp sensor_msgs cv_bridge)
target_link_libraries(mono ${LIBS})

add_executable(rgbd
  src/rgbd/rgbd.cpp
  src/rgbd/rgbd-slam-node.cpp
)
ament_target_dependencies(rgbd rclcpp sensor_msgs cv_bridge message_filters)
target_link_libraries(rgbd ${LIBS})

add_executable(stereo
  src/stereo/stereo.cpp
  src/stereo/stereo-slam-node.cpp
)
ament_target_dependencies(stereo rclcpp sensor_msgs cv_bridge message_filters)
target_link_libraries(stereo ${LIBS})

add_executable(stereo-inertial
  src/stereo-inertial/stereo-inertial.cpp
  src/stereo-inertial/stereo-inertial-node.cpp
)
ament_target_dependencies(stereo-inertial rclcpp sensor_msgs cv_bridge)
target_link_libraries(stereo-inertial ${LIBS})

install(TARGETS mono rgbd stereo stereo-inertial
  DESTINATION lib/${PROJECT_NAME})

# Install launch files.
#install(DIRECTORY launch config vocabulary
#  DESTINATION share/${PROJECT_NAME}/)

ament_package()

