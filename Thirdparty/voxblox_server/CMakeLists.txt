cmake_minimum_required(VERSION 2.8.3)
project(voxblox_server)

set(CPP_STANDARD_VERSION "17" CACHE STRING "Desired C++ standard version") 

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

#SET(CMAKE_BUILD_TYPE Debug)

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

if(CMAKE_BUILD_TYPE STREQUAL "Release")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native -fPIC -DNDEBUG")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3 -march=native -fPIC -DNDEBUG")
else()
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -g -fPIC")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -g -fPIC")
endif()

set(CMAKE_POSITION_INDEPENDENT_CODE ON) # NOTE: -fPIC above does not work with static libraries

# Set the C++ standard 
set(CMAKE_CXX_STANDARD ${CPP_STANDARD_VERSION})
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_definitions(-DCOMPILE_WITHOUT_ROS)

#find_package(catkin REQUIRED COMPONENTS roscpp std_msgs sensor_msgs geometry_msgs tf open_chisel pcl_ros chisel_msgs)

# get cmake modules from main dir 
LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/../../cmake_modules)


find_package(Eigen3 3.1.0 REQUIRED)

find_package(PCL 1.7 REQUIRED )
add_definitions( ${PCL_DEFINITIONS} )
link_directories( ${PCL_LIBRARY_DIRS})

find_package(Protobuf REQUIRED)


include_directories(
include
${EIGEN3_INCLUDE_DIR}
${PCL_INCLUDE_DIRS}
${PROJECT_SOURCE_DIR}/../voxblox/include
)

set(EXTERNAL_LIBS 
${EIGEN3_LIBS}
${PCL_LIBRARIES}
${PROJECT_SOURCE_DIR}/../voxblox/lib/libvoxblox.a
${PROJECT_SOURCE_DIR}/../voxblox/lib/libvoxblox_proto.a
${PROTOBUF_LIBRARIES}
)


#############
# LIBRARIES #
#############
add_library(${PROJECT_NAME}
  src/esdf_server.cc
  #src/interactive_slider.cc
  src/simulation_server.cc
  src/transformer.cc
  src/tsdf_server.cc
)

target_link_libraries(${PROJECT_NAME} ${EXTERNAL_LIBS})

############
# BINARIES #
############

#cs_add_executable(voxblox_eval
  #src/voxblox_eval.cc
#)
#target_link_libraries(voxblox_eval ${PROJECT_NAME})

#cs_add_executable(tsdf_server
  #src/tsdf_server_node.cc
#)
#target_link_libraries(tsdf_server ${PROJECT_NAME})

#cs_add_executable(esdf_server
  #src/esdf_server_node.cc
#)
#target_link_libraries(esdf_server ${PROJECT_NAME})


#cs_add_executable(simulation_eval
  #src/simulation_eval.cc
#)
#target_link_libraries(simulation_eval ${PROJECT_NAME})

#cs_add_executable(visualize_tsdf
  #src/visualize_tsdf.cc
#)
#target_link_libraries(visualize_tsdf ${PROJECT_NAME})

##########
# EXPORT #
##########
#cs_install()
#cs_export()


## install 

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)


