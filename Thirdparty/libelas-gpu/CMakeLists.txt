cmake_minimum_required(VERSION 2.8.8)
project(libelas_gpu)

set(CPP_STANDARD_VERSION "17" CACHE STRING "Desired C++ standard version") # need C++17 given the following error "nvcc fatal   : Value 'c++20' is not defined for option 'std'"
set(WITH_GPU ON CACHE BOOL "Added GPU support")
set(BUILD_WITH_MARCH_NATIVE ON  CACHE BOOL "Build with \"-march native\"")

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

#SET(CMAKE_BUILD_TYPE Debug)  # for enabling debugging 

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

# Include libraries
find_package(CUDA)
if (CUDA_FOUND AND WITH_GPU)
    set(USE_CUDA true)     # N.B.: here opencv libs are not used!
    MESSAGE("USE_CUDA ${USE_CUDA}")
else()
    set(CUDA_LIBRARIES "")
    MESSAGE("CUDA NOT FOUND!")
endif()

# This option allows the generations of a file compile_commands.json in our build folder: that file contains the full command line to compile individual source files
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Set the C++ standard 
set(CMAKE_CXX_STANDARD ${CPP_STANDARD_VERSION})
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Enable compile optimizations
if(CMAKE_BUILD_TYPE STREQUAL "Release")
    #set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -march=native ")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -pthread -ftree-vectorize -funroll-loops -fPIC")
    # Enable SSE3 cpu commands
    # If we are on ARM we need to find an alternative
    #set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -msse -msse2 -msse3")
else()
    # Enable debug flags
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g3 -Wall -fPIC")
endif()

if(BUILD_WITH_MARCH_NATIVE)
  set(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS} -march=native") 
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native")         
endif()

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

if(USE_CUDA)

    # Set our nvcc flags
    # http://stackoverflow.com/a/13244930
    #set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} -arch sm_20 -gencode arch=compute_20,code=sm_20)

    set(CUDA_PROPAGATE_HOST_FLAGS OFF)
    include_directories(${CUDA_INCLUDE_DIRS})
    # Set files to build
    set(sources_gpu
      GPU/elas_gpu.cu
    )

    #set(CUDA_ARCH_BIN "35 50 52 53 61 62 75 80" CACHE STRING "Specify 'real' GPU arch to build binaries for, BIN(PTX) format is supported. Example: 1.3 2.1(1.3) or 13 21(13)")
    cuda_select_nvcc_arch_flags(AUTO_DETECTED_ARCHS)
    set(CUDA_ARCH_BIN ${AUTO_DETECTED_ARCHS} CACHE STRING "Autodetected CUDA architectures")
    message(STATUS "AUTO_DETECTED_ARCHS: ${AUTO_DETECTED_ARCHS}")

    set(CUDA_ARCH_PTX "" CACHE STRING "Specify 'virtual' PTX arch to build PTX intermediate code for. Example: 1.0 1.2 or 10 12")

    include(${CMAKE_MODULE_PATH}/CudaComputeTargetFlags.cmake)
    APPEND_TARGET_ARCH_FLAGS()

    set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} "-std=c++${CMAKE_CXX_STANDARD} -use_fast_math")
    set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} "-Xcompiler;-fPIC;")

    if(CMAKE_BUILD_TYPE STREQUAL "Release")
        set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} "-O3 ")
    else()
        set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} "-g ")        
    endif()

    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DUSE_CUDA")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DUSE_CUDA")

endif()


############
# INCLUDE  #
############

set(LIBELAS_INCLUDE_DIRS
${PROJECT_SOURCE_DIR}/CPU
${PROJECT_SOURCE_DIR}/GPU
)

include_directories(
${LIBELAS_INCLUDE_DIRS}
)


# Include all source files in each sub-directory
# file(GLOB_RECURSE sources_cpu CPU/*.cpp)
# file(GLOB_RECURSE sources_gpu GPU/*.cu)
# file(GLOB_RECURSE sources_cpu GPU_test/*.cpp)


# Set files the build
set(sources_cpu
  CPU/descriptor.cpp
  CPU/elas.cpp
  CPU/filter.cpp
  CPU/matrix.cpp
  CPU/triangle.cpp
  GPU/ElasInterface.cpp
)



if(USE_CUDA )

    # convert the gpu code to cpu code
    cuda_compile(sources_gpu_built ${sources_gpu})

    add_library(${PROJECT_NAME}
    ${sources_gpu_built}
    ${sources_cpu}
    )

    target_link_libraries(${PROJECT_NAME}
    ${CUDA_LIBRARIES}
    )

else()

    add_library(${PROJECT_NAME}
    ${sources_cpu}
    )

endif()


# testing CPU binary
add_executable(test_cpu main_cpu.cpp)
target_link_libraries(test_cpu ${PROJECT_NAME})


# Testing binary
add_executable(main_test main_test.cpp)
target_link_libraries(main_test ${PROJECT_NAME})

if(USE_CUDA )

    # Testing GPU binary
    add_executable(test_gpu main_gpu.cpp)
    target_link_libraries(test_gpu ${PROJECT_NAME} ${CUDA_LIBRARIES})

endif()

# Testing Interface binary
add_executable(test_interface main_interface.cpp)
target_link_libraries(test_interface ${PROJECT_NAME} ${CUDA_LIBRARIES})


## install

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)


# ### create symbolic links
set(MY_COMMAND sh)
set(MY_ARG ${CMAKE_CURRENT_SOURCE_DIR}/create_include.sh ${LIBELAS_INCLUDE_DIRS})
execute_process(COMMAND ${MY_COMMAND} ${MY_ARG} WORKING_DIRECTORY ${PROJECT_SOURCE_DIR} RESULT_VARIABLE RES)


message(STATUS "CMAKE_CXX_FLAGS: ${CMAKE_CXX_FLAGS}") 
message(STATUS "CUDA_NVCC_FLAGS: ${CUDA_NVCC_FLAGS}")
