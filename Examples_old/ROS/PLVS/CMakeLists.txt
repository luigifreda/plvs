cmake_minimum_required(VERSION 2.8.3)
project(plvs)

set(CPP_STANDARD_VERSION "17" CACHE STRING "Desired C++ standard version") 
set(OPENCV_VERSION "4" CACHE STRING "Desired OpenCV version") # this is the first target version that is searched for 
set(BUILD_WITH_MARCH_NATIVE ON  CACHE BOOL "Build with \"-march native\"")
set(WITH_OPENMP             ON CACHE BOOL "Added OpenMP support") # useful for PCL (since we use the macro PCL_NO_PRECOMPILE!)
set(WITH_CUDA               OFF CACHE BOOL "Added CUDA support")
set(WITH_LOCAL_PCL          OFF  CACHE BOOL "Add local PCL support")

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

#====================================

# cmake modules

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/../../../cmake_modules)

include(${CMAKE_MODULE_PATH}/CheckFastfusionSupport.cmake)
if(BUILD_FASTFUSION)
    message(STATUS "building fastfusion ***************")
else()
    message(STATUS "fastfusion disabled! ***************")
endif()



find_program(LSB_RELEASE lsb_release)
message("LSB_RELEASE: ${LSB_RELEASE}")

find_package(Eigen3 REQUIRED)

# set(Pangolin_DIR ${PROJECT_SOURCE_DIR}/../../../Thirdparty/Pangolin/build/src)
set(Pangolin_DIR ${PROJECT_SOURCE_DIR}/../../../Thirdparty/Pangolin/build/)
message(STATUS "set custom pangolin folder: ${Pangolin_DIR}")
find_package(Pangolin REQUIRED)
message(STATUS "pangolin include: ${Pangolin_INCLUDE_DIRS}")


if(WITH_LOCAL_PCL)
    set(CMAKE_PREFIX_PATH ${PROJECT_SOURCE_DIR}/../../../Thirdparty/pcl/install/share/pcl)
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

set(PGK_DEPS
  cv_bridge
  nav_msgs
  roscpp
  sensor_msgs
  tf
  tf2_ros
  pcl_ros
)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
	${PGK_DEPS}
)



## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   nav_msgs#   sensor_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS  include
#  LIBRARIES plvs
  CATKIN_DEPENDS
	${PGK_DEPS}
 # DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
${catkin_INCLUDE_DIRS}
${OpenCV_INCLUDE_DIRS}
${PCL_INCLUDE_DIRS}
${GLOG_INCLUDE_DIRS}
${CUDA_INCLUDE_DIRS}
)

include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/../../../
${PROJECT_SOURCE_DIR}/../../../include
${PROJECT_SOURCE_DIR}/../../../include/CameraModels
${PROJECT_SOURCE_DIR}/../../../Thirdparty/line_descriptor/include
${PROJECT_SOURCE_DIR}/../../../Thirdparty/Sophus
${Pangolin_INCLUDE_DIRS}
)

link_directories(${CUDA_LIBRARY_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})

add_definitions( ${PCL_DEFINITIONS} )
link_directories( ${PCL_LIBRARY_DIRS})

if(BUILD_FASTFUSION)
set(FASTFUSION_LIBS
    ${PROJECT_SOURCE_DIR}/../../../Thirdparty/fastfusion/lib/libauxiliary.a
    ${PROJECT_SOURCE_DIR}/../../../Thirdparty/fastfusion/lib/libcamerautils.a
    ${PROJECT_SOURCE_DIR}/../../../Thirdparty/fastfusion/lib/libgeometryfusion_aos.a
    ${PROJECT_SOURCE_DIR}/../../../Thirdparty/fastfusion/lib/libgeometryfusion_mipmap_cpu.a
    )
endif()


set(LIBS
${PROJECT_SOURCE_DIR}/../../../lib/lib${PROJECT_NAME}.so
#${PROJECT_SOURCE_DIR}/../../../lib/lib${PROJECT_NAME}_pc_mapping.so
${PROJECT_SOURCE_DIR}/../../../Thirdparty/DBoW2/lib/libDBoW2.so
${PROJECT_SOURCE_DIR}/../../../Thirdparty/g2o/lib/libg2o.so
${PROJECT_SOURCE_DIR}/../../../Thirdparty/volumetric_mapping/lib/libvolumetric_mapping.so
${PROJECT_SOURCE_DIR}/../../../Thirdparty/chisel_server/lib/libchisel_server.a
${PROJECT_SOURCE_DIR}/../../../Thirdparty/open_chisel/lib/libopen_chisel.a
${PROJECT_SOURCE_DIR}/../../../Thirdparty/voxblox_server/lib/libvoxblox_server.a
${PROJECT_SOURCE_DIR}/../../../Thirdparty/voxblox/lib/libvoxblox.a
${PROJECT_SOURCE_DIR}/../../../Thirdparty/voxblox/lib/libvoxblox_proto.a
${PROJECT_SOURCE_DIR}/../../../Thirdparty/line_descriptor/lib/liblinedesc.a
${OpenCV_LIBS}
${EIGEN3_LIBS}
${Pangolin_LIBRARIES}
${FASTFUSION_LIBS}
${GLOG_LIBRARIES}
${PROTOBUF_LIBRARIES}
${CUDA_LIBRARIES}
${Boost_LIBRARIES}
)



## Declare a C++ library
# add_library(orb_slam2
#   src/${PROJECT_NAME}/orb_slam2.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(orb_slam2 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
# add_executable(orb_slam2_node src/orb_slam2_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(orb_slam2_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(orb_slam2_node
#   ${catkin_LIBRARIES}
# )


# Node for monocular camera
add_executable(Mono src/ros_mono.cc)
target_link_libraries(Mono ${catkin_LIBRARIES} ${LIBS})
add_dependencies(Mono ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# Node for monocular camera (Augmented Reality Demo)
# add_executable(MonoAR
# src/AR/ros_mono_ar.cc
# src/AR/ViewerAR.h
# src/AR/ViewerAR.cc
# )
# target_link_libraries(MonoAR ${catkin_LIBRARIES} ${LIBS})
# add_dependencies(MonoAR ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


# Node for stereo camera
add_executable(Stereo src/ros_stereo.cc)
target_link_libraries(Stereo ${catkin_LIBRARIES} ${LIBS})
add_dependencies(Stereo ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# Node for RGB-D camera
#add_executable(RGBD_original src/ros_rgbd_original.cc)
#target_link_libraries(RGBD_original ${catkin_LIBRARIES} ${LIBS})
#add_dependencies(RGBD_original ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# Node for RGB-D camera
add_executable(RGBD src/ros_rgbd.cc)
target_link_libraries(RGBD ${catkin_LIBRARIES} ${LIBS})
add_dependencies(RGBD ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# Node for monocular-inertial camera
add_executable(Mono_Inertial src/ros_mono_inertial.cc)
target_link_libraries(Mono_Inertial ${catkin_LIBRARIES} ${LIBS})
add_dependencies(Mono_Inertial ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# Node for stereo-inertial camera
add_executable(Stereo_Inertial src/ros_stereo_inertial.cc)
target_link_libraries(Stereo_Inertial ${catkin_LIBRARIES} ${LIBS})
add_dependencies(Stereo_Inertial ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# Node for ZED camera
#add_executable(ZED src/zed.cc)
#target_link_libraries(ZED ${catkin_LIBRARIES} ${LIBS})
#add_dependencies(ZED ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# Node for R200 camera
#add_executable(R200 src/r200.cc)
#target_link_libraries(R200 ${catkin_LIBRARIES} ${LIBS})
#add_dependencies(R200 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS orb_slam2 orb_slam2_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_orb_slam2.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
