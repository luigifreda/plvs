cmake_minimum_required(VERSION 2.8)
project(volometric_mapping)

set(CPP_STANDARD_VERSION "17" CACHE STRING "Desired C++ standard version") 
set(OPENCV_VERSION "3" CACHE STRING "Desired OpenCV version") # this is the first target version that is searched for 
set(BUILD_WITH_MARCH_NATIVE ON  CACHE BOOL "Build with \"-march native\"")
set(WITH_LOCAL_PCL          OFF  CACHE BOOL "Add local PCL support")

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

#SET(CMAKE_BUILD_TYPE Debug)

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

if(CMAKE_BUILD_TYPE STREQUAL "Release")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall -O3 -fPIC -DNDEBUG")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3 -fPIC -DNDEBUG")
else()
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall -g -fPIC")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -g -fPIC")
endif()

if(BUILD_WITH_MARCH_NATIVE)
  set(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS} -march=native") 
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native")         
endif()

# Set the C++ standard 
set(CMAKE_CXX_STANDARD ${CPP_STANDARD_VERSION})
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# This option allows the generations of a file compile_commands.json in our build folder: that file contains the full command line to compile individual source files
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# get cmake modules from main dir 
LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/../../cmake_modules)

add_definitions(-DCOMPILE_WITHOUT_ROS)

find_package(Eigen3 3.1.0 REQUIRED)

find_package(GLOG REQUIRED)
if(NOT GLOG_FOUND)
	message(FATAL_ERROR "please run: sudo apt-get install libgoogle-glog-dev " )
endif()

find_package(octomap REQUIRED)
message(STATUS "octomap include: ${OCTOMAP_INCLUDE_DIRS}")
message(STATUS "octomap lib dirs: ${OCTOMAP_LIBRARY_DIRS}")
message(STATUS "octomap libs: ${OCTOMAP_LIBRARIES}")

message(STATUS "Desired OpenCV version: ${OPENCV_VERSION}")
find_package(OpenCV ${OPENCV_VERSION} REQUIRED)
message(STATUS "found OpenCV version: ${OpenCV_VERSION}")
message(STATUS "opencv include: ${OpenCV_INCLUDE_DIRS}")
message(STATUS "opencv lib dirs: ${OpenCV_INSTALL_PATH}")
message(STATUS "opencv libs: ${OpenCV_LIBS}")


if(WITH_LOCAL_PCL)
    set(CMAKE_PREFIX_PATH ${PROJECT_SOURCE_DIR}/../pcl/install/share/pcl)
endif()
find_package(PCL 1.7 REQUIRED)
message(STATUS "pcl include: ${PCL_INCLUDE_DIRS}")
message(STATUS "pcl libs: ${PCL_LIBRARY_DIRS}")
message(STATUS "pcl definitions: ${PCL_DEFINITIONS}")
add_definitions( ${PCL_DEFINITIONS} )
link_directories( ${PCL_LIBRARY_DIRS})


set(VOLUMETRIC_MAPPING_INCLUDE_DIRS
${PROJECT_SOURCE_DIR}/volumetric_mapping/volumetric_map_base/include
${PROJECT_SOURCE_DIR}/volumetric_mapping/octomap_world/include
${PROJECT_SOURCE_DIR}/volumetric_mapping/minkindr/include
)

include_directories(
${VOLUMETRIC_MAPPING_INCLUDE_DIRS}
${EIGEN3_INCLUDE_DIR}
${OCTOMAP_INCLUDE_DIRS}
${OpenCV_INCLUDE_DIRS}
${PCL_INCLUDE_DIRS}
${GLOG_INCLUDE_DIRS}
)

link_directories(
${PCL_LIBRARY_DIRS}
${OCTOMAP_LIBRARY_DIRS}
#${GLOG_LIBRARIES}
)

set(SRCS_VOLUMETRIC_MAPPING
  ${PROJECT_SOURCE_DIR}/volumetric_mapping/volumetric_map_base/src/world_base.cc
  ${PROJECT_SOURCE_DIR}/volumetric_mapping/octomap_world/src/octomap_world.cc
)

add_library(volumetric_mapping SHARED ${SRCS_VOLUMETRIC_MAPPING})
target_link_libraries(volumetric_mapping ${OCTOMAP_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES})


## install

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# ### create symbolic links
set(MY_COMMAND sh)
set(MY_ARG ${CMAKE_CURRENT_SOURCE_DIR}/create_include.sh ${VOLUMETRIC_MAPPING_INCLUDE_DIRS})
execute_process(COMMAND ${MY_COMMAND} ${MY_ARG} WORKING_DIRECTORY ${PROJECT_SOURCE_DIR} RESULT_VARIABLE RES)


