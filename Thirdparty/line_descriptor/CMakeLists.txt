cmake_minimum_required(VERSION 2.8)
project( line-descriptor )

set(CPP_STANDARD_VERSION "17" CACHE STRING "Desired C++ standard version") 
set(OPENCV_VERSION "3" CACHE STRING "Desired OpenCV version") # this is the first target version that is searched for 
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

message(STATUS "Desired OpenCV version: ${OPENCV_VERSION}")
find_package(OpenCV ${OPENCV_VERSION} REQUIRED)
message(STATUS "found OpenCV version: ${OpenCV_VERSION}")
message(STATUS "opencv include: ${OpenCV_INCLUDE_DIRS}")
message(STATUS "opencv lib dirs: ${OpenCV_INSTALL_PATH}")
message(STATUS "opencv libs: ${OpenCV_LIBS}")

#if(COMMAND cmake_policy)
#  	cmake_policy(SET CMP0003 NEW)
#endif(COMMAND cmake_policy)

link_directories(${OpenCV_LIBS_DIR})
include_directories(${OpenCV_INCLUDE_DIRS})

SET(BUILD_SHARED_LIBS OFF)

SET(CMAKE_MODULE_PATH $ENV{CMAKE_MODULE_PATH})


set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/build)
set(LIBRARY_OUTPUT_PATH    ${PROJECT_SOURCE_DIR}/lib)

include_directories( include ${OpenCV_INCLUDE_DIRS} )
list(APPEND LINK_LIBS ${OpenCV_LIBS} )
file(GLOB_RECURSE all_include_files RELATIVE "${CMAKE_SOURCE_DIR}" *.h *.hpp)

link_directories(${CMAKE_SOURCE_DIR}/src/)
file(GLOB_RECURSE all_source_files RELATIVE "${CMAKE_SOURCE_DIR}src/" *.cpp )

add_custom_target( linedesc_includes DEPENDS ${all_include_files} SOURCES ${all_source_files} )

add_library( linedesc ${all_source_files} )
target_link_libraries( linedesc ${LINK_LIBS} )
#set_property(TARGET linedesc PROPERTY POSITION_INDEPENDENT_CODE ON)

