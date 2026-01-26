cmake_minimum_required(VERSION 3.1)
project(libSGM VERSION 2.0.0)

set(CPP_STANDARD_VERSION "17" CACHE STRING "Desired C++ standard version") 
option(ENABLE_ZED_DEMO "Build a Demo using ZED Camera" OFF)
option(ENABLE_SAMPLES  "Build samples" OFF)
option(LIBSGM_SHARED   "Build a shared library" OFF)
set(BUILD_WITH_MARCH_NATIVE ON  CACHE BOOL "Build with \"-march native\"")

if(WIN32)
  set(ZED_SDK_LIB "C:\\Program Files (x86)\\ZED SDK\\lib\\sl_zed64.lib" CACHE STRING "ZED SDK library(sl_zed**.llb) path.")
  set(ZED_SDK_INCLUDE_DIR "C:\\Program Files (x86)\\ZED SDK\\include" CACHE STRING "ZED SDK include path.")
else()
  set(ZED_SDK_LIB "/usr/local/zed/lib/libsl_zed.so" CACHE STRING "ZED SDK library(sl_zed**.llb) path.")
  set(ZED_SDK_INCLUDE_DIR "/usr/local/zed/include" CACHE STRING "ZED SDK include path.")
endif()

# This option allows the generations of a file compile_commands.json in our build folder: that file contains the full command line to compile individual source files
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# set(CMAKE_CXX_STANDARD 11)
# set(CMAKE_CXX_EXTENSIONS OFF)

# Set the C++ standard 
set(CMAKE_CXX_STANDARD ${CPP_STANDARD_VERSION})
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -O3 -fPIC -DNDEBUG")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3 -fPIC -DNDEBUG")

if(BUILD_WITH_MARCH_NATIVE)
  set(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS} -march=native") 
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native")         
endif()


find_package(CUDA REQUIRED)

option(LIBSGM_AUTO_CUDA_ARCH "Auto-detect CUDA architecture" ON)
set(CUDA_ARCH "" CACHE STRING "NVCC arch flags (e.g. -arch=sm_86 or -gencode...)")

if(LIBSGM_AUTO_CUDA_ARCH AND (CUDA_ARCH STREQUAL "" OR CUDA_ARCH STREQUAL "-arch=sm_50"))
  if(COMMAND cuda_select_nvcc_arch_flags)
    cuda_select_nvcc_arch_flags(_libsgm_auto_arch)
    if(_libsgm_auto_arch)
      set(CUDA_ARCH "${_libsgm_auto_arch}" CACHE STRING "NVCC arch flags (e.g. -arch=sm_86 or -gencode...)" FORCE)
      message(STATUS "Auto-detected CUDA arch flags: ${CUDA_ARCH}")
    endif()
  endif()

  if(CUDA_ARCH STREQUAL "")
    execute_process(
      COMMAND nvidia-smi --query-gpu=compute_cap --format=csv,noheader
      OUTPUT_VARIABLE _libsgm_cc_raw
      OUTPUT_STRIP_TRAILING_WHITESPACE
      ERROR_QUIET
    )
    if(_libsgm_cc_raw)
      string(REPLACE "\n" ";" _libsgm_cc_list "${_libsgm_cc_raw}")
      list(GET _libsgm_cc_list 0 _libsgm_cc_first)
      string(REPLACE "." "" _libsgm_cc_nodot "${_libsgm_cc_first}")
      if(NOT _libsgm_cc_nodot STREQUAL "")
        set(CUDA_ARCH "-arch=sm_${_libsgm_cc_nodot}" CACHE STRING "NVCC arch flags (e.g. -arch=sm_86 or -gencode...)" FORCE)
        message(STATUS "Detected CUDA compute capability ${_libsgm_cc_first} -> ${CUDA_ARCH}")
      endif()
    endif()
  endif()

  if(CUDA_ARCH STREQUAL "")
    set(CUDA_ARCH "-arch=sm_50" CACHE STRING "NVCC arch flags (e.g. -arch=sm_86 or -gencode...)")
    message(WARNING "Could not auto-detect CUDA arch; using ${CUDA_ARCH}. Set CUDA_ARCH manually to override.")
  endif()
endif()

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
