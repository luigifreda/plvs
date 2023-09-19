cmake_minimum_required(VERSION 2.8.3)
project(chisel_server)

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

#find_package(catkin REQUIRED COMPONENTS roscpp std_msgs sensor_msgs geometry_msgs tf open_chisel pcl_ros chisel_msgs)

# get cmake modules from main dir 
LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/../../cmake_modules)


find_package(Eigen3 3.1.0 REQUIRED)

find_package(PCL 1.7 REQUIRED )
add_definitions(-DPCL_NO_PRECOMPILE)  # this is strictly required since we defined a new pcl point type
add_definitions( ${PCL_DEFINITIONS} )
link_directories( ${PCL_LIBRARY_DIRS})


include_directories(
include
${EIGEN3_INCLUDE_DIR}
${PCL_INCLUDE_DIRS}
${PROJECT_SOURCE_DIR}/../open_chisel/include
)

set(EXTERNAL_LIBS 
${EIGEN3_LIBS}
${PCL_LIBRARIES}
${PROJECT_SOURCE_DIR}/../open_chisel/lib/libopen_chisel.a
pthread
)

add_library(${PROJECT_NAME} src/ChiselServer.cpp)
target_link_libraries(${PROJECT_NAME} ${EXTERNAL_LIBS})

add_executable(ChiselNode src/ChiselNode.cpp)
target_link_libraries(ChiselNode ${PROJECT_NAME} ${EXTERNAL_LIBS})

## install 

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#install(DIRECTORY include/${PROJECT_NAME}/
#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#  PATTERN ".git" EXCLUDE
#)
