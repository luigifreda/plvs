cmake_minimum_required(VERSION 3.1)
project(libSGM VERSION 2.0.0)

set(CPP_STANDARD_VERSION "17" CACHE STRING "Desired C++ standard version") 
option(ENABLE_ZED_DEMO "Build a Demo using ZED Camera" OFF)
option(ENABLE_SAMPLES  "Build samples" OFF)
option(LIBSGM_SHARED   "Build a shared library" OFF)

if(WIN32)
  set(ZED_SDK_LIB "C:\\Program Files (x86)\\ZED SDK\\lib\\sl_zed64.lib" CACHE STRING "ZED SDK library(sl_zed**.llb) path.")
  set(ZED_SDK_INCLUDE_DIR "C:\\Program Files (x86)\\ZED SDK\\include" CACHE STRING "ZED SDK include path.")
else()
  set(ZED_SDK_LIB "/usr/local/zed/lib/libsl_zed.so" CACHE STRING "ZED SDK library(sl_zed**.llb) path.")
  set(ZED_SDK_INCLUDE_DIR "/usr/local/zed/include" CACHE STRING "ZED SDK include path.")
endif()


# set(CMAKE_CXX_STANDARD 11)
# set(CMAKE_CXX_EXTENSIONS OFF)

# Set the C++ standard 
set(CMAKE_CXX_STANDARD ${CPP_STANDARD_VERSION})
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(CUDA REQUIRED)

set(CUDA_ARCH "-arch=sm_50" CACHE STRING "Value of the NVCC -arch option.") # N.B.: the package does not compile if the architecture is changed

configure_file(${CMAKE_SOURCE_DIR}/include/libsgm_config.h.in
               ${CMAKE_SOURCE_DIR}/include/libsgm_config.h
)

add_subdirectory(src)

if(ENABLE_SAMPLES)
    add_subdirectory(sample/image)
    add_subdirectory(sample/movie)
    add_subdirectory(sample/benchmark)
endif()

if(ENABLE_ZED_DEMO)
	add_subdirectory(sample/zed)
endif()



message(STATUS "CMAKE_CXX_FLAGS: ${CMAKE_CXX_FLAGS}") 
message(STATUS "CUDA_NVCC_FLAGS: ${CUDA_NVCC_FLAGS}")
