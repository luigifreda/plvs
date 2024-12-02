cmake_minimum_required(VERSION 2.8)
project(fastfusion)

set(CPP_STANDARD_VERSION "17" CACHE STRING "Desired C++ standard version") 
set(OPENCV_VERSION "3" CACHE STRING "Desired OpenCV version") # this is the first target version that is searched for 
set(BUILD_WITH_MARCH_NATIVE ON  CACHE BOOL "Build with \"-march native\"")

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
#set(CMAKE_BUILD_TYPE RelWithDebInfo)
#set(ROS_BUILD_TYPE RelWithDebInfo)
#set(CMAKE_BUILD_TYPE Debug)
#set(ROS_BUILD_TYPE Debug)

# This option allows the generations of a file compile_commands.json in our build folder: that file contains the full command line to compile individual source files
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Set the C++ standard 
set(CMAKE_CXX_STANDARD ${CPP_STANDARD_VERSION})
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if (CMAKE_COMPILER_IS_GNUCC)
    execute_process(COMMAND ${CMAKE_C_COMPILER} -dumpversion
                    OUTPUT_VARIABLE GCC_VERSION)
    string(REGEX MATCHALL "[0-9]+" GCC_VERSION_COMPONENTS ${GCC_VERSION})
    list(GET GCC_VERSION_COMPONENTS 0 GCC_MAJOR)
    #list(GET GCC_VERSION_COMPONENTS 1 GCC_MINOR)

    message(STATUS "GCC Major Version is " ${GCC_MAJOR})
    message(STATUS "GCC Major Version is " ${GCC_MINOR})
endif()

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -O3 -ftree-vectorize")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3 -ftree-vectorize")

if(BUILD_WITH_MARCH_NATIVE)
  # NOTE: This project independently manages specific SSE options below
  set(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS} -march=native") 
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native")         
endif()

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -frounding-math")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -frounding-math")

execute_process(COMMAND cat /proc/cpuinfo OUTPUT_VARIABLE CPU_INFO)
string(REGEX MATCHALL "avx2" AVX_STRING ${CPU_INFO})
list(LENGTH AVX_STRING AVX_STRING_LENGTH)
#message(STATUS "AVX_STRING: " ${AVX_STRING})
message(STATUS "Number of avx2 occurrences in /proc/cpuinfo: " ${AVX_STRING_LENGTH})


if( ${AVX_STRING_LENGTH} GREATER 0 )
    message(STATUS "Compiling with AVX2 support")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -mavx2 -funroll-loops -DOWNAVX")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -mavx2 -funroll-loops -DOWNAVX")
else()
    message(STATUS "Compiling without AVX2 support")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -msse4.2 -funroll-loops")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -msse4.2 -funroll-loops")
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC -fopenmp")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC -fopenmp")

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DPRINT_DEBUG")
#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DPRINT_DEBUG")

add_definitions(-DOPENCV_TRAITS_ENABLE_DEPRECATED) # necessary for fastfusion compiled with Opencv3


# get cmake modules from main dir 
LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/../../cmake_modules)

#FIND_PACKAGE(Qt4 REQUIRED)

message(STATUS "Desired OpenCV version: ${OPENCV_VERSION}")
find_package(OpenCV ${OPENCV_VERSION} REQUIRED)
message(STATUS "found OpenCV version: ${OpenCV_VERSION}")
message(STATUS "opencv include: ${OpenCV_INCLUDE_DIRS}")
message(STATUS "opencv lib dirs: ${OpenCV_INSTALL_PATH}")
message(STATUS "opencv libs: ${OpenCV_LIBS}")

FIND_PACKAGE(Eigen3 3.1.0 REQUIRED)

#FIND_PACKAGE(QGLViewerQt4)

find_package(Boost COMPONENTS thread system)
#find_package(TBB)

include_directories(
${PROJECT_SOURCE_DIR}/src
${EIGEN3_INCLUDE_DIR}
${OpenCV_INCLUDE_DIRS}
${QGLVIEWER_INCLUDE_DIR}
${Boost_INCLUDE_DIRS}
)

set(EXTERNAL_LIBS
${OpenCV_LIBS}
${EIGEN3_LIBS}
)



## install

set(FUSION_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/src)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# ### create symbolic links
set(MY_COMMAND sh)
set(MY_ARG ${CMAKE_CURRENT_SOURCE_DIR}/create_include.sh ${FUSION_INCLUDE_DIRS})
execute_process(COMMAND ${MY_COMMAND} ${MY_ARG} WORKING_DIRECTORY ${PROJECT_SOURCE_DIR} RESULT_VARIABLE RES)

# subdirs

add_subdirectory(src)






