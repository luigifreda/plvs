cmake_minimum_required(VERSION 2.8)
project(open_chisel)
 
set(CPP_STANDARD_VERSION "17" CACHE STRING "Desired C++ standard version") 

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

#SET(CMAKE_BUILD_TYPE Debug) # for forcing Debug

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

if(CMAKE_BUILD_TYPE STREQUAL "Release")

    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native -fPIC -DNDEBUG -pthread")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3 -march=native -fPIC -DNDEBUG -pthread")

else()

    #set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -g -fPIC -pthread")
    #set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -g -fPIC -pthread")

    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -g -O3 -march=native -fPIC -pthread")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -g -O3 -march=native -fPIC -pthread")

endif()

set(CMAKE_POSITION_INDEPENDENT_CODE ON) # NOTE: -fPIC above does not work with static libraries

# Set the C++ standard 
set(CMAKE_CXX_STANDARD ${CPP_STANDARD_VERSION})
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# get cmake modules from main dir 
LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/../../cmake_modules)

find_package(Eigen3 3.1.0 REQUIRED)

find_package(PCL 1.7 REQUIRED )
add_definitions( ${PCL_DEFINITIONS} )
link_directories( ${PCL_LIBRARY_DIRS})


include_directories(
include
${EIGEN3_INCLUDE_DIR}
${PCL_INCLUDE_DIRS}
)

set(EXTERNAL_LIBS 
${EIGEN3_LIBS}
${PCL_LIBRARIES}
)


add_library(${PROJECT_NAME} 
	src/Chunk.cpp 
	src/ChunkManager.cpp 
	src/DistVoxel.cpp  
	src/ColorVoxel.cpp
	src/geometry/AABB.cpp
	src/geometry/Plane.cpp
	src/geometry/Frustum.cpp
	src/camera/Intrinsics.cpp
	src/camera/PinholeCamera.cpp
	src/pointcloud/PointCloud.cpp
	src/ProjectionIntegrator.cpp
	src/Chisel.cpp
	src/mesh/Mesh.cpp
	src/marching_cubes/MarchingCubes.cpp
	src/io/PLY.cpp
	src/geometry/Raycast.cpp)

target_link_libraries(${PROJECT_NAME} $EXTERNAL_LIBS})

## install 

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#install(DIRECTORY include/${PROJECT_NAME}/
#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#)
