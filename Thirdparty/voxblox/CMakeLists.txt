cmake_minimum_required(VERSION 2.8.3)
project(voxblox)

set(CPP_STANDARD_VERSION "17" CACHE STRING "Desired C++ standard version") 
set(BUILD_WITH_MARCH_NATIVE ON  CACHE BOOL "Build with \"-march native\"")

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

#SET(CMAKE_BUILD_TYPE Debug)

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

# This option allows the generations of a file compile_commands.json in our build folder: that file contains the full command line to compile individual source files
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_BUILD_TYPE STREQUAL "Release")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -O3 -fPIC -DNDEBUG -pthread")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3 -fPIC -DNDEBUG -pthread")
else()
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -g -O3 -fPIC -pthread")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -g -O3 -fPIC -pthread")
endif()

if(BUILD_WITH_MARCH_NATIVE)
  set(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS} -march=native") 
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native")         
endif()

set(CMAKE_POSITION_INDEPENDENT_CODE ON) # NOTE: -fPIC above does not work with static libraries

# Set the C++ standard 
set(CMAKE_CXX_STANDARD ${CPP_STANDARD_VERSION})
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# get cmake modules from main dir 
LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/../../cmake_modules)

find_package(Eigen3 3.1.0 REQUIRED)

find_package(GLOG REQUIRED)
if(NOT GLOG_FOUND)
	message(FATAL_ERROR "please run: sudo apt-get install libgoogle-glog-dev " )
endif()


############
# PROTOBUF #
############
# General idea: first check if we have protobuf catkin, then use that.
# Otherwise use system protobuf.
set(PROTO_DEFNS proto/voxblox/Block.proto
                proto/voxblox/Layer.proto)
set(ADDITIONAL_LIBRARIES "")


message(STATUS "Using system protobuf")
find_package(Protobuf REQUIRED)
include_directories(${PROTOBUF_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_BINARY_DIR})

PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ${PROTO_DEFNS})
set(ADDITIONAL_LIBRARIES ${PROTOBUF_LIBRARY})



############
# INCLUDE  #
############

set(VOXBLOX_INCLUDE_DIRS
${PROJECT_SOURCE_DIR}/include
${PROJECT_SOURCE_DIR}/minkindr/include
)

include_directories(
${VOXBLOX_INCLUDE_DIRS}
${EIGEN3_INCLUDE_DIR}
${GLOG_INCLUDE_DIRS}
)

#link_directories(
#)

set(EXTERNAL_LIBS
${EIGEN3_LIBS}
${GLOG_LIBRARIES}
${PROTOBUF_LIBRARIES}
)

####################
# SET SOURCE FILES #
####################

set("${PROJECT_NAME}_SRCS"
  src/core/block.cc
  src/core/esdf_map.cc
  src/core/tsdf_map.cc
  src/integrator/esdf_integrator.cc
  src/integrator/esdf_occ_integrator.cc
  src/integrator/integrator_utils.cc
  src/integrator/tsdf_integrator.cc
  src/io/mesh_ply.cc
  src/io/sdf_ply.cc
  src/mesh/marching_cubes.cc
  src/simulation/objects.cc
  src/simulation/simulation_world.cc
  src/utils/camera_model.cc
  src/utils/evaluation_utils.cc
  src/utils/layer_utils.cc
  src/utils/protobuf_utils.cc
  src/utils/timing.cc
  src/utils/voxel_utils.cc
)

#############
# LIBRARIES #
#############
# NOTE(mereweth@jpl.nasa.gov) - Be careful when compiling Proto cc files. It is
# best to do so only once (make a shared library for each set of Proto files).
# Otherwise, at some point, you will get errors from double-adding protobuf
# formats, of the following form:
# [libprotobuf ERROR google/protobuf/descriptor_database.cc:57]
# File already exists in database: Block.proto

# Avoid having multiple compiled copies of the same .pb.cc
# file sharing a single copy of libprotobuf.so
add_library(${PROJECT_NAME}_proto
  ${PROTO_SRCS}
)
target_link_libraries(${PROJECT_NAME}_proto ${PROTOBUF_LIBRARIES})

add_library(${PROJECT_NAME}
  ${${PROJECT_NAME}_SRCS}
)
target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}_proto ${PROTOBUF_LIBRARIES})


############
# BINARIES #
############

add_executable(tsdf_to_esdf
  test/tsdf_to_esdf.cc
)
target_link_libraries(tsdf_to_esdf ${PROJECT_NAME} ${PROJECT_NAME}_proto ${PROTOBUF_LIBRARIES} ${EXTERNAL_LIBS})

add_executable(test_load_esdf
  test/test_load_esdf.cc
)
target_link_libraries(test_load_esdf ${PROJECT_NAME} ${PROJECT_NAME}_proto ${PROTOBUF_LIBRARIES} ${EXTERNAL_LIBS})

#########
# TESTS #
#########
add_custom_target(test_data)
add_custom_command(TARGET test_data
                  COMMAND rm -rf test_data
                  COMMAND mkdir -p test_data
                  COMMAND cp -r ${CMAKE_SOURCE_DIR}/test/test_data/*
                                test_data/ || :)

#add_definitions(-DVISUALIZE_UNIT_TEST_RESULTS)

#catkin_add_gtest(test_approx_hash_array
  #test/test_approx_hash_array.cc
#)
#target_link_libraries(test_approx_hash_array ${PROJECT_NAME})

#catkin_add_gtest(test_tsdf_map
  #test/test_tsdf_map.cc
#)
#target_link_libraries(test_tsdf_map ${PROJECT_NAME})

#catkin_add_gtest(test_protobuf
  #test/test_protobuf.cc
#)
#target_link_libraries(test_protobuf ${PROJECT_NAME})

#catkin_add_gtest(test_tsdf_interpolator
  #test/test_tsdf_interpolator.cc
#)
#target_link_libraries(test_tsdf_interpolator ${PROJECT_NAME})

#catkin_add_gtest(test_layer
  #test/test_layer.cc
#)
#target_link_libraries(test_layer ${PROJECT_NAME})

#catkin_add_gtest(test_merge_integration
  #test/test_merge_integration.cc
#)
#target_link_libraries(test_merge_integration ${PROJECT_NAME} ${catkin_LIBRARIES})

#catkin_add_gtest(test_layer_utils
  #test/test_layer_utils.cc
#)
#target_link_libraries(test_layer_utils ${PROJECT_NAME})

##########
# EXPORT #
##########
#cs_install()
#cs_export(INCLUDE_DIRS include ${CMAKE_CURRENT_BINARY_DIR}
#          CFG_EXTRAS voxblox-extras.cmake
#          LIBRARIES ${ADDITIONAL_LIBRARIES})

## install

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# ### create symbolic links
set(MY_COMMAND sh)
set(MY_ARG ${CMAKE_CURRENT_SOURCE_DIR}/create_include.sh ${VOXBLOX_INCLUDE_DIRS})
execute_process(COMMAND ${MY_COMMAND} ${MY_ARG} WORKING_DIRECTORY ${PROJECT_SOURCE_DIR} RESULT_VARIABLE RES)
