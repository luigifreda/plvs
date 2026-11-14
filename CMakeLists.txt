cmake_minimum_required(VERSION 2.8)
project(plvs)

#====================================
# options

set(CPP_STANDARD_VERSION "17" CACHE STRING "Desired C++ standard version") # we need c++17 since nvcc does not support c++20 (probably we can try mixing standards and let nvcc use c++17 ... not sure about the result)
set(OPENCV_VERSION "4" CACHE STRING "Desired OpenCV version") # this is the first target version that is searched for 

set(BUILD_WITH_MARCH_NATIVE ON  CACHE BOOL "Build with \"-march native\"")

set(WITH_FASTFUSION     ON  CACHE BOOL "Add fastfusion support") # for enabling RGBD dense reconstruction via fastfusion (it needs Intel SIMD)
set(WITH_G2O_NEW        OFF CACHE BOOL "Add g2o new support") # for using the new version of g2o with Cholmod and Csparse solvers (and more...)
set(WITH_LIBELAS        ON  CACHE BOOL "Add libelas support")  # for stereo dense reconstruction (it needs Intel SIMD, optionally can be compiled with CUDA)
set(WITH_LIBSGM         ON  CACHE BOOL "Add stereo libsgm support")  # for stereo dense reconstruction (it needs CUDA)
set(WITH_OPENMP         ON  CACHE BOOL "Add OpenMP support") # useful for PCL (since we use the macro PCL_NO_PRECOMPILE!)
set(WITH_CUDA           OFF CACHE BOOL "Add CUDA support")
set(USE_HIP             OFF CACHE BOOL "Build with HIP for AMD GPUs")
set(WITH_TRACY_PROFILER OFF CACHE BOOL "Add Tracy profiler support.")
set(WITH_RERUN          OFF CACHE BOOL "Add Rerun support.")
set(WITH_LOCAL_PCL      OFF  CACHE BOOL "Add local PCL support")

if(WITH_CUDA AND USE_HIP)
    message(FATAL_ERROR "WITH_CUDA and USE_HIP cannot both be enabled. Select one GPU backend.")
endif()

#====================================
# c++ standard 

# Set the C++ standard 
set(CMAKE_CXX_STANDARD ${CPP_STANDARD_VERSION})
set(CMAKE_CXX_STANDARD_REQUIRED ON)

#====================================
# general cmake options 

# NOTE: this is to remove the tons of cmake warning coming from importing ext packages 
# see here: https://github.com/PointCloudLibrary/pcl/issues/3680
# when this is fixed, we can remove the following 3 lines.
if(NOT DEFINED CMAKE_SUPPRESS_DEVELOPER_WARNINGS)
     set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS 1 CACHE INTERNAL "No dev warnings")
endif()

# This option allows the generations of a file compile_commands.json in our build folder: that file contains the full command line to compile individual source files
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#====================================
# compiler and build options 

if(WIN32)
    # A shared plvs would need every templated PointCloud* class member explicitly
    # exported (__declspec(dllexport)); the project has no such annotations, so on
    # Windows build the core as a static library and let each executable pull the
    # template instantiations it needs.
    set(LIBRARY_TYPE STATIC)
else()
    set(LIBRARY_TYPE SHARED)
endif()
#set(LIBRARY_TYPE STATIC)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

#set(CMAKE_BUILD_TYPE Debug)  # force Debug 

message("Build type: " ${CMAKE_BUILD_TYPE})

#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall   -O3")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3")
#set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -march=native")
#set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -march=native")

if(CMAKE_BUILD_TYPE STREQUAL "Release")

    # Best release. clang for x86_64-pc-windows-msvc rejects -fPIC (implicit on
    # Windows); keep it on the other platforms.
    if(WIN32)
        set(MY_FLAGS "-Wall -O3 -DNDEBUG -Wno-unused-parameter -Wno-unused-function")
    else()
        set(MY_FLAGS "-Wall -O3 -fPIC -DNDEBUG -Wno-unused-parameter -Wno-unused-function") # -Wextra -Wno-deprecated
    endif()

    if(BUILD_WITH_MARCH_NATIVE)
        set(MY_FLAGS "${MY_FLAGS} -march=native")      
    endif()

    #set(MY_FLAGS "-Wall -Wextra -O3 -pthread -ftree-vectorize -funroll-loops")

    # Best release Nvidia Jetson TX2
    #set(MY_FLAGS "-Wall -O3 -ftree-vectorize -funroll-loops -pthread -Wextra")

    if(WITH_TX2)
        # Best release Nvidia Jetson TX2
        #set(MY_FLAGS "-Wall -O3 -march=armv8-a+crypto -mcpu=cortex-a57+crypto -Wextra -ffast-math -flto -fPIC -DNDEBUG")
    endif()
else()

    message(AUTHOR_WARNING "\n!!!Compiling in Debug mode!!!\n")
    # Debug
    #set(MY_FLAGS "-Wall -O3  -g -pthread")

    # Debug with g2o compiled with -march=native; this should avoid g2o crashes (see "Mismatched build flags" on https://github.com/ethz-asl/eigen_catkin/wiki/Eigen-Memory-Issues#memory-misalignment )
    if(WIN32)
        set(MY_FLAGS "-Wall  -g -O3")  # clang rejects -fPIC/-pthread for windows-msvc
    else()
        set(MY_FLAGS "-Wall  -g -O3 -fPIC -pthread") # -fsanitize=address
    endif()

    if(BUILD_WITH_MARCH_NATIVE)
        set(MY_FLAGS "${MY_FLAGS} -march=native")      
    endif()

    # this makes g2o crash if g2o is compiled with optimizations 
    #set(MY_FLAGS "-Wall  -g")
endif()

# Add OpenMP flags
if( WITH_OPENMP )
    set(MY_FLAGS "${MY_FLAGS} -fopenmp")   
endif()

# get and store all the active flags 
set(MY_C_FLAGS "${CMAKE_C_FLAGS} ${MY_FLAGS}")
set(MY_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${MY_FLAGS}")    

# apply flags
set(CMAKE_C_FLAGS "${MY_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${MY_CXX_FLAGS}")

if(WIN32)
    # NOMINMAX: keep windows.h from defining min/max macros (they clash with STL
    # and PCL/Eigen). WIN32_LEAN_AND_MEAN: avoid windows.h pulling the legacy
    # winsock1 headers, which conflict with <winsock2.h>. _USE_MATH_DEFINES: M_PI.
    add_compile_definitions(NOMINMAX WIN32_LEAN_AND_MEAN _USE_MATH_DEFINES)
    # Tracking keeps whole Frame objects (~420 KB each) on the stack, and the
    # monocular initialization path nests three such frames, which overflows the
    # 1 MB Windows default stack. Reserve the 8 MB that Linux threads get.
    add_link_options(LINKER:/STACK:8388608)
endif()

message(STATUS "CMAKE_CXX_FLAGS: ${CMAKE_CXX_FLAGS}")

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

add_definitions(-DCOMPILEDWITHC11)  #TODO: replace the macro everywhere with COMPILEDWITHC14 or completely remove it

set(SHADERS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/Shaders" CACHE PATH "Where the shaders live")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DSHADERS_DIR=${SHADERS_DIR}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DSHADERS_DIR=${SHADERS_DIR}")

#====================================
# cmake modules

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

#====================================
# CUDA 
message(STATUS "WITH_CUDA: " ${WITH_CUDA})

if (WITH_CUDA)
    find_package(CUDA REQUIRED)

    set(CUDA_PROPAGATE_HOST_FLAGS OFF)
    SET(CUDA_HOST_COMPILER /usr/bin/g++)

    include_directories(${CUDA_INCLUDE_DIRS})

    # Set files to build
    set(CUDA_SOURCES
        src/cuda/Allocator_gpu.cu
        src/cuda/Fast_gpu.cu
        src/cuda/Orb_gpu.cu
        src/cuda/Cuda.cu
    )

    option(PLVS_AUTO_CUDA_ARCH "Auto-detect CUDA arch list" ON)
    set(CUDA_ARCH_BIN "" CACHE STRING "Specify 'real' GPU arch to build binaries for, BIN(PTX) format is supported. Example: 1.3 2.1(1.3) or 13 21(13)")
    set(CUDA_ARCH_PTX "" CACHE STRING "Specify 'virtual' PTX arch to build PTX intermediate code for. Example: 1.0 1.2 or 10 12")

    if(PLVS_AUTO_CUDA_ARCH AND CUDA_ARCH_BIN STREQUAL "")
        execute_process(
            COMMAND nvidia-smi --query-gpu=compute_cap --format=csv,noheader
            OUTPUT_VARIABLE _plvs_cc_raw
            OUTPUT_STRIP_TRAILING_WHITESPACE
            ERROR_QUIET
        )
        if(_plvs_cc_raw)
            string(REPLACE "\n" ";" _plvs_cc_list "${_plvs_cc_raw}")
            set(_plvs_arch_bins "")
            foreach(_plvs_cc IN LISTS _plvs_cc_list)
                string(REPLACE "." "" _plvs_cc_nodot "${_plvs_cc}")
                if(NOT _plvs_cc_nodot STREQUAL "")
                    list(APPEND _plvs_arch_bins "${_plvs_cc_nodot}")
                endif()
            endforeach()
            list(REMOVE_DUPLICATES _plvs_arch_bins)
            if(_plvs_arch_bins)
                set(CUDA_ARCH_BIN "${_plvs_arch_bins}" CACHE STRING "Specify 'real' GPU arch to build binaries for, BIN(PTX) format is supported. Example: 1.3 2.1(1.3) or 13 21(13)" FORCE)
                message(STATUS "Detected CUDA_ARCH_BIN: ${CUDA_ARCH_BIN}")
            endif()
        endif()
        if(CUDA_ARCH_BIN STREQUAL "")
            message(WARNING "Could not auto-detect CUDA_ARCH_BIN; set it manually to enable -gencode flags.")
        endif()
    endif()

    include(${CMAKE_MODULE_PATH}/CudaComputeTargetFlags.cmake)
    APPEND_TARGET_ARCH_FLAGS()

    set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} "-std=c++${CMAKE_CXX_STANDARD} -use_fast_math")
    set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} "-Xcompiler;-fPIC;")
    #set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} "--ftz=true;--prec-div=false;--prec-sqrt=false")

    if(CMAKE_BUILD_TYPE STREQUAL "Release")
        set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} "-O3 ")
    else()
        set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} "-g ")        
    endif()

    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DUSE_CUDA")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DUSE_CUDA")
endif()

#====================================
# HIP (AMD GPU support)
message(STATUS "USE_HIP: " ${USE_HIP})

if(USE_HIP)
    if(CMAKE_VERSION VERSION_LESS 3.21)
        message(FATAL_ERROR "USE_HIP requires CMake 3.21 or newer.")
    endif()
    # enable_language(HIP) honors an explicit -DCMAKE_HIP_ARCHITECTURES, otherwise
    # auto-detects the host GPU(s), and errors on a no-GPU build host.
    enable_language(HIP)
    message(STATUS "CMAKE_HIP_ARCHITECTURES: ${CMAKE_HIP_ARCHITECTURES}")

    # Set files to build
    set(HIP_SOURCES
        src/cuda/Allocator_gpu.cu
        src/cuda/Fast_gpu.cu
        src/cuda/Orb_gpu.cu
        src/cuda/Cuda.cu
    )

    # Mark .cu files as HIP language
    set_source_files_properties(${HIP_SOURCES} PROPERTIES LANGUAGE HIP)

    # Add USE_HIP define and include the compat header path
    add_compile_definitions(USE_HIP)

    # The device sources include <cuda/helper_cuda.h> for checkCudaErrors, which
    # the vendored CUDA samples header only defines when the CUDA toolkit has
    # already defined __DRIVER_TYPES_H__, so it is unavailable here. hip_compat/
    # holds substitutes named after the headers they stand in for; prepending it
    # makes those includes resolve here on a ROCm build, while the vendored
    # copies stay untouched and keep serving the CUDA build.
    include_directories(BEFORE ${PROJECT_SOURCE_DIR}/hip_compat)

    # Host C++ translation units that include OpenCV's cv::cuda headers (e.g.
    # cuda_stream_accessor.hpp) must select the HIP runtime branch and resolve
    # hip/hip_runtime.h. Define __HIP_PLATFORM_AMD__ and add the ROCm include path
    # for the host compiler so those headers do not fall back to <cuda_runtime.h>.
    # Derive a ROCm include hint from the HIP compiler location (its bin dir sits
    # under the ROCm root) so this resolves on Windows where ROCm is not at
    # /opt/rocm; fall back to ROCM_PATH and the Linux default.
    get_filename_component(_hip_bin "${CMAKE_HIP_COMPILER}" DIRECTORY)
    get_filename_component(_rocm_root "${_hip_bin}/../../.." ABSOLUTE)
    find_path(HIP_INCLUDE_DIR hip/hip_runtime.h
        HINTS "${_rocm_root}/include" "$ENV{ROCM_PATH}/include" /opt/rocm/include)
    add_compile_definitions(__HIP_PLATFORM_AMD__)
    include_directories(${HIP_INCLUDE_DIR})

    # Host TUs reach OpenCV cv::cuda headers and the project's cuda/{Fast,Orb}.hpp
    # that use CUDA runtime spellings (cudaStream_t, ...). Force-include the
    # CUDA->HIP alias shim into every TU so those resolve to the HIP runtime types.
    add_compile_options(-include ${PROJECT_SOURCE_DIR}/src/cuda/cuda_to_hip.h)

    # HIP compile options
    set(CMAKE_HIP_FLAGS "${CMAKE_HIP_FLAGS} -std=c++${CMAKE_CXX_STANDARD}")
    if(CMAKE_BUILD_TYPE STREQUAL "Release")
        set(CMAKE_HIP_FLAGS "${CMAKE_HIP_FLAGS} -O3")
    else()
        set(CMAKE_HIP_FLAGS "${CMAKE_HIP_FLAGS} -g")
    endif()
endif()

#====================================
# packages 

find_package(Eigen3 3.1.0 REQUIRED)
message(STATUS "eigen include: ${EIGEN3_INCLUDE_DIR}")
# The local FindEigen3 module only sets EIGEN3_INCLUDE_DIR; some dependency config
# packages (e.g. Pangolin) additionally require the Eigen3::Eigen imported target
# from Eigen's own config package, so pull it in when available.
if(NOT TARGET Eigen3::Eigen)
    find_package(Eigen3 CONFIG QUIET)
endif()


find_package(Boost REQUIRED COMPONENTS thread system serialization)


# Check fastfusion support and SIMD supports
include(${CMAKE_MODULE_PATH}/CheckFastfusionSupport.cmake)
# CheckFastfusionSupport sets BUILD_FASTFUSION from CPU SIMD alone; honor the
# user-facing WITH_FASTFUSION toggle so the optional backend can be turned off.
if(NOT WITH_FASTFUSION)
    set(BUILD_FASTFUSION FALSE)
endif()
if(BUILD_FASTFUSION AND WITH_FASTFUSION)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DUSE_FASTFUSION")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DUSE_FASTFUSION")
    message(STATUS "Building fastfusion! ***************")
else()
    message(STATUS "fastfusion disabled! ***************")
endif()

if(NOT HAVE_SSE3)
    set(WITH_LIBELAS OFF)
    message(STATUS "libelas disabled! ***************")
endif()


message(STATUS "Desired OpenCV version: ${OPENCV_VERSION}")
find_package(OpenCV ${OPENCV_VERSION} REQUIRED)
message(STATUS "found OpenCV version: ${OpenCV_VERSION}")
message(STATUS "opencv include: ${OpenCV_INCLUDE_DIRS}")
message(STATUS "opencv lib dirs: ${OpenCV_INSTALL_PATH}")
message(STATUS "opencv libs: ${OpenCV_LIBS}")


# On Linux the project builds Pangolin in-tree (Thirdparty/Pangolin/build). On
# Windows, take Pangolin from a CMake config package (e.g. vcpkg) via the normal
# find_package search unless the caller pins Pangolin_DIR explicitly.
if(NOT WIN32)
    set(Pangolin_DIR ${PROJECT_SOURCE_DIR}/Thirdparty/Pangolin/build/) # this seems to work in general
endif()
#set(CMAKE_PREFIX_PATH ${PROJECT_SOURCE_DIR}/Thirdparty/Pangolin/install/lib/cmake/Pangolin/) # this does not work (after make install) if we set the RPATH!
#set(Pangolin_DIR ${PROJECT_SOURCE_DIR}/Thirdparty/Pangolin/install/) # this does not work (after make install) if we set the RPATH!
message(STATUS "set custom pangolin folder: ${Pangolin_DIR}")
find_package(Pangolin REQUIRED)
message(STATUS "pangolin include: ${Pangolin_INCLUDE_DIRS}")
message(STATUS "pangolin libs: ${Pangolin_LIBRARY}")


find_package(realsense2)


if(WITH_LOCAL_PCL)
    set(CMAKE_PREFIX_PATH ${PROJECT_SOURCE_DIR}/Thirdparty/pcl/install/share/pcl)
endif()
find_package(PCL 1.7 REQUIRED)
message(STATUS "pcl include: ${PCL_INCLUDE_DIRS}")
message(STATUS "pcl libs: ${PCL_LIBRARY_DIRS}")
message(STATUS "pcl definitions: ${PCL_DEFINITIONS}")
add_definitions(-DPCL_NO_PRECOMPILE)  # this is strictly required since we defined a new pcl point type
if(WITH_CUDA)
    # N.B.: the lines of this block allow to compile CUDA with PCL without troubles
    get_directory_property(dir_defs DIRECTORY ${CMAKE_SOURCE_DIR} COMPILE_DEFINITIONS)
    set(vtk_flags)
    foreach(it ${dir_defs})
        if(it MATCHES "vtk*")
        list(APPEND vtk_flags ${it})
        endif()
    endforeach()
    foreach(d ${vtk_flags})
        remove_definitions(-D${d})
    endforeach()
endif()
add_definitions( ${PCL_DEFINITIONS} )
link_directories( ${PCL_LIBRARY_DIRS})


find_package(GLOG REQUIRED)
if(NOT GLOG_FOUND)
	message(FATAL_ERROR "please run: sudo apt-get install libgoogle-glog-dev " )
endif()



find_package(octomap REQUIRED)
message(STATUS "octomap include: ${OCTOMAP_INCLUDE_DIRS}")
message(STATUS "octomap lib dirs: ${OCTOMAP_LIBRARY_DIRS}")
message(STATUS "octomap libs: ${OCTOMAP_LIBRARIES}")


find_package(Protobuf REQUIRED)
# The modern protobuf (e.g. vcpkg's, where it is built static against Abseil)
# exposes the protobuf::libprotobuf imported target carrying its Abseil link
# dependencies; the legacy PROTOBUF_LIBRARIES variable is just the bare library
# path and omits them, leaving unresolved absl symbols at the final link. Prefer
# the target when present.
if(WIN32 AND TARGET protobuf::libprotobuf)
    # On Windows with vcpkg static protobuf the bare PROTOBUF_LIBRARIES path
    # omits Abseil link dependencies; use the imported target which carries them.
    # On Linux the system shared protobuf's FindProtobuf sets PROTOBUF_LIBRARIES to
    # the bare .so path; using the conda-found imported target would pull
    # /opt/conda/include first and break the generated .pb.h version check.
    set(PROTOBUF_LIBRARIES protobuf::libprotobuf)
    # The static protobuf also needs utf8_range (utf8_range_IsValid / ValidPrefix),
    # which its imported target does not always carry transitively; add it.
    find_package(utf8_range CONFIG QUIET)
    foreach(_u utf8_range::utf8_validity utf8_range::utf8_range)
        if(TARGET ${_u})
            list(APPEND PROTOBUF_LIBRARIES ${_u})
        endif()
    endforeach()
endif()
message(STATUS "protobuf libs: ${PROTOBUF_LIBRARIES}")

# When the dependency stack is the vcpkg static Abseil (Windows static-md),
# protobuf and glog reference Abseil's SwissTable / hashtablez internals across
# several archives that single-pass static linking will not resolve unless the
# full Abseil target set is on the link line. Collect every imported absl:: target
# (read from its exported targets file, as Abseil imports them GLOBAL) into
# ABSEIL_LIBRARIES so the closure is complete. Empty where Abseil is absent
# (e.g. the Linux build with a shared protobuf/glog), and also with a DLL Abseil
# (absl::abseil_dll), whose import library already resolves every symbol and
# whose test-only targets name libraries that are not installed.
set(ABSEIL_LIBRARIES "")
if(WIN32 AND NOT TARGET absl::abseil_dll)
    file(GLOB _absl_target_files "${CMAKE_PREFIX_PATH}/share/absl/abslTargets.cmake")
    foreach(_f IN LISTS _absl_target_files)
        file(STRINGS "${_f}" _lines REGEX "add_library\\(absl::")
        foreach(_l IN LISTS _lines)
            string(REGEX MATCH "absl::[A-Za-z0-9_]+" _n "${_l}")
            if(_n AND TARGET ${_n})
                list(APPEND ABSEIL_LIBRARIES ${_n})
            endif()
        endforeach()
    endforeach()
    list(REMOVE_DUPLICATES ABSEIL_LIBRARIES)
endif()


find_package(LibCrypto REQUIRED)


if(WIN32)
    # pkg-config is not the norm on Windows; use GLFW's CMake config package
    # (e.g. from vcpkg), which exposes the glfw target and its include dirs.
    find_package(glfw3 CONFIG REQUIRED)
    set(GLFW_LIBRARIES glfw)
    set(GLFW_INCLUDE_DIRS "")
else()
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(GLFW REQUIRED glfw3)
endif()


if(BUILD_FASTFUSION)
    set(FASTFUSION_INCLUDES 
    ${PROJECT_SOURCE_DIR}/Thirdparty/fastfusion/include
    )
    set(FASTFUSION_LIBRARIES
    ${PROJECT_SOURCE_DIR}/Thirdparty/fastfusion/lib/libauxiliary.a
    ${PROJECT_SOURCE_DIR}/Thirdparty/fastfusion/lib/libcamerautils.a
    ${PROJECT_SOURCE_DIR}/Thirdparty/fastfusion/lib/libgeometryfusion_aos.a
    ${PROJECT_SOURCE_DIR}/Thirdparty/fastfusion/lib/libgeometryfusion_mipmap_cpu.a
    )
endif()


if(WITH_LIBELAS)
    if(NOT CUDA_FOUND)
        find_package(CUDA) # make the check if not already done above (CUDA are required if libelas )
    endif()
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DUSE_LIBELAS")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DUSE_LIBELAS")
    if(CUDA_FOUND)
        message(STATUS "building with libelas CUDA ***************")
    elseif(USE_HIP)
        message(STATUS "building with libelas HIP ***************")
    else()
        message(STATUS "building with libelas CPU ***************")
    endif()
    set(LIBELAS_INCLUDES 
    ${PROJECT_SOURCE_DIR}/Thirdparty/libelas-gpu/include
    )
    if(WIN32)
        set(LIBELAS_LIBRARIES ${PROJECT_SOURCE_DIR}/Thirdparty/libelas-gpu/lib/libelas_gpu.lib)
    else()
        set(LIBELAS_LIBRARIES ${PROJECT_SOURCE_DIR}/Thirdparty/libelas-gpu/lib/liblibelas_gpu.a)  # this contains or not CUDA
    endif()
else()
    message(STATUS "libelas disabled! ***************")
endif()


if(WITH_LIBSGM)
    if(NOT CUDA_FOUND AND NOT USE_HIP)
        find_package(CUDA) # make the check if not already done above
    endif()
    if(CUDA_FOUND OR USE_HIP)
        set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DUSE_LIBSGM")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DUSE_LIBSGM")
        message(STATUS "building with stereo libsgm GPU ***************")
        set(LIBSGM_INCLUDES
        ${PROJECT_SOURCE_DIR}/Thirdparty/libsgm/include
        )
        if(WIN32)
            set(LIBSGM_LIBRARIES ${PROJECT_SOURCE_DIR}/Thirdparty/libsgm/lib/sgm.lib)  # device code: HIP
        else()
            set(LIBSGM_LIBRARIES ${PROJECT_SOURCE_DIR}/Thirdparty/libsgm/lib/libsgm.a)  # device code: CUDA or HIP
        endif()
    else()
        message(STATUS "stereo libsgm disabled! ***************")
    endif()
else()
    message(STATUS "stereo libsgm disabled! ***************")
endif()


if( WITH_G2O_NEW )
    # In order to set RPATH (instead of RUNPATH) for g2o and other libraries linking issue.
    # ISSUE: g2o binaries link to system g2o (here /opt/ros/noetic/lib/libg2o_core.so) and crash (different version); 
    # we need to set LD_LIBRARY_PATH for correct linking
    # https://stackoverflow.com/questions/47117443/dynamic-linking-with-rpath-not-working-under-ubuntu-17-10 
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wl,--disable-new-dtags")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wl,--disable-new-dtags")          
    find_package(Cholmod)
    include_directories(${CHOLMOD_INCLUDE_DIR})        
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DUSE_G2O_NEW")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DUSE_G2O_NEW")    
    # NOTE: use the following block of lines to test latest g2o version support.
    #       If it crashes. Check this link: https://github.com/rst-tu-dortmund/teb_local_planner/issues/336
    set(CMAKE_PREFIX_PATH ${PROJECT_SOURCE_DIR}/Thirdparty/g2o_new/install/lib/cmake/g2o/)
    set(G2O_ROOT ${PROJECT_SOURCE_DIR}/Thirdparty/g2o_new/install/)
    message(STATUS "set custom g2o root folder: ${G2O_ROOT}")
    find_package(G2O REQUIRED)
    # used in the latest g2o version 
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DG2O_USE_VENDORED_CERES")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DG2O_USE_VENDORED_CERES")
else()
    # g2o provided with ORBSLAM3
    set(G2O_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/)
    if(WIN32)
        # g2o places its DLL + import lib in bin/ on Windows (see its CMakeLists)
        set(G2O_LIBS ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/bin/g2o.lib)
    else()
        set(G2O_LIBS ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so)
    endif()
endif()
message(STATUS "g2o include: ${G2O_INCLUDE_DIR}")
message(STATUS "g2o lib dirs: ${G2O_LIB_DIR}")
message(STATUS "g2o libs: ${G2O_LIBS}")

if(WITH_TRACY_PROFILER)
  if(EXISTS "/usr/local/lib/libTracyClient.a")
    message(STATUS "Found Tracy")
    add_definitions(-DTRACY_ENABLE)
    set(TRACY_LIBS /usr/local/lib/libTracyClient.a -ldl -lpthread)
  endif()
endif()

if(WITH_RERUN)
    # rerun_sdk
    add_definitions(-DRERUN_ENABLED) 
    set(CMAKE_PREFIX_PATH ${PROJECT_SOURCE_DIR}/Thirdparty/rerun/install/lib/cmake/rerun_sdk/)
    set(Rerun_DIR ${PROJECT_SOURCE_DIR}/Thirdparty/rerun/install/)
    message(STATUS "set custom rerun folder: ${Rerun_DIR}")
    find_package(rerun_sdk REQUIRED)
    message(STATUS "rerun include: ${RERUN_LIBRARIES}")
endif()

#====================================
# includes 

include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/include
${PROJECT_SOURCE_DIR}/include/CameraModels
${G2O_INCLUDE_DIR}
${PROJECT_SOURCE_DIR}/Thirdparty/volumetric_mapping/include
${PROJECT_SOURCE_DIR}/Thirdparty/open_chisel/include
${PROJECT_SOURCE_DIR}/Thirdparty/chisel_server/include
${FASTFUSION_INCLUDES}
${PROJECT_SOURCE_DIR}/Thirdparty/voxblox/include
${PROJECT_SOURCE_DIR}/Thirdparty/voxblox_server/include
${PROJECT_SOURCE_DIR}/Thirdparty/line_descriptor/include
${LIBELAS_INCLUDES}
${LIBSGM_INCLUDES}
${PROJECT_SOURCE_DIR}/Thirdparty/Sophus
${EIGEN3_INCLUDE_DIR}
${Pangolin_INCLUDE_DIRS}
${PCL_INCLUDE_DIRS}
${GLOG_INCLUDE_DIRS}
${OCTOMAP_INCLUDE_DIRS}
${OpenCV_INCLUDE_DIRS}
${Boost_INCLUDE_DIRS}
${GLFW_INCLUDE_DIRS}
${CHOLMOD_INCLUDE_DIR}
)

# If RealSense SDK is found the library is added and its examples compiled
if(realsense2_FOUND)
    include_directories(${PROJECT_NAME} ${realsense_INCLUDE_DIR})
endif()

#====================================
# libs 

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)


# On Windows, Boost libraries are versioned (e.g. boost_serialization-vc143-mt-...
# .lib), so the bare boost_serialization / boost_thread names do not resolve; rely
# on the targets find_package(Boost) populated in Boost_LIBRARIES instead.
if(WIN32)
    set(BOOST_EXTRA_LIBS Boost::serialization Boost::thread)
else()
    set(BOOST_EXTRA_LIBS boost_serialization boost_thread)
endif()
set(EXTERNAL_CORE_LIBS   # core external libs
${OpenCV_LIBS}
${EIGEN3_LIBS}
${Boost_LIBRARIES} ${BOOST_EXTRA_LIBS}
#Boost::boost # NOTE this does not work!
${Pangolin_LIBRARIES}
${G2O_LIBS}
${PCL_LIBRARIES}
${GLOG_LIBRARIES}
${OCTOMAP_LIBRARIES}
${PROTOBUF_LIBRARIES}
${ABSEIL_LIBRARIES}
${LibCrypto_LIBRARY}
${GLFW_LIBRARIES}
${CHOLMOD_LIBRARIES}
)
if(CUDA_FOUND)
    set(EXTERNAL_CORE_LIBS ${EXTERNAL_CORE_LIBS} ${CUDA_LIBRARIES}) # ${CUDA_LIBRARIES} are used just for stereo disparity computation when libelas or libsgm support are added
endif()
if(USE_HIP)
    # libsgm.a / liblibelas_gpu.a carry HIP device code; link the HIP runtime.
    # The runtime import lib lives under the ROCm root (derived from the HIP
    # compiler) on Windows, or /opt/rocm/lib(64) on Linux.
    get_filename_component(_hip_bin "${CMAKE_HIP_COMPILER}" DIRECTORY)
    get_filename_component(_rocm_root "${_hip_bin}/../../.." ABSOLUTE)
    find_library(HIP_RUNTIME_LIB amdhip64
        HINTS "${_rocm_root}/lib" "$ENV{ROCM_PATH}/lib" /opt/rocm/lib /opt/rocm/lib64)
    set(EXTERNAL_CORE_LIBS ${EXTERNAL_CORE_LIBS} ${HIP_RUNTIME_LIB})
endif()
if(WITH_RERUN)
    set(EXTERNAL_CORE_LIBS ${EXTERNAL_CORE_LIBS} rerun_sdk)
endif()

# If RealSense SDK is found the library is added and its examples compiled
if(realsense2_FOUND)
    message(STATUS "found REALSENSE LIB: ${realsense2_LIBRARY}")
    #target_link_libraries(${PROJECT_NAME} ${realsense2_LIBRARY})
    set(EXTERNAL_CORE_LIBS ${EXTERNAL_CORE_LIBS} ${realsense2_LIBRARY})
endif()


# The bundled Thirdparty libraries are referenced by output-file path. On Windows
# the all-clang toolchain emits MSVC-style names (an import .lib for each SHARED
# DLL, a .lib for each static archive) instead of lib*.so / lib*.a; select the
# right names per platform.
set(TP ${PROJECT_SOURCE_DIR}/Thirdparty)
if(WIN32)
    set(EXTERNAL_LIBS  # Thirdparty libs
    ${TP}/DBoW2/lib/DBoW2.lib
    ${TP}/line_descriptor/lib/linedesc.lib
    ${TP}/volumetric_mapping/lib/volumetric_mapping.lib
    ${TP}/chisel_server/lib/chisel_server.lib
    ${TP}/open_chisel/lib/open_chisel.lib
    ${FASTFUSION_LIBRARIES}
    ${TP}/voxblox/lib/voxblox.lib
    ${TP}/voxblox/lib/voxblox_proto.lib
    ${TP}/voxblox_server/lib/voxblox_server.lib
    ${LIBELAS_LIBRARIES}
    ${LIBSGM_LIBRARIES}
    )
else()
set(EXTERNAL_LIBS  # Thirdparty libs
${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
${PROJECT_SOURCE_DIR}/Thirdparty/line_descriptor/lib/liblinedesc.a
${PROJECT_SOURCE_DIR}/Thirdparty/volumetric_mapping/lib/libvolumetric_mapping.so
${PROJECT_SOURCE_DIR}/Thirdparty/chisel_server/lib/libchisel_server.a
${PROJECT_SOURCE_DIR}/Thirdparty/open_chisel/lib/libopen_chisel.a
${FASTFUSION_LIBRARIES}
${PROJECT_SOURCE_DIR}/Thirdparty/voxblox/lib/libvoxblox.a
${PROJECT_SOURCE_DIR}/Thirdparty/voxblox/lib/libvoxblox_proto.a
${PROJECT_SOURCE_DIR}/Thirdparty/voxblox_server/lib/libvoxblox_server.a
${LIBELAS_LIBRARIES}
${LIBSGM_LIBRARIES}
)
endif()

link_directories(
  ${G2O_LIB_DIR}  
  ${PCL_LIBRARY_DIRS}
)

#====================================
# source files and folders to build 

#add_library(${PROJECT_NAME} SHARED
set(CPU_SOURCES
src/WinBoostArchiveCompat.cpp
src/System.cc
src/Tracking.cc
src/LocalMapping.cc
src/LoopClosing.cc
src/ORBextractor.cc
src/ORBmatcher.cc
src/FrameDrawer.cc
src/Converter.cc
src/MapPoint.cc
src/KeyFrame.cc
src/Atlas.cc
src/Map.cc
src/MapDrawer.cc
src/Optimizer.cc
src/Frame.cc
src/KeyFrameDatabase.cc
src/Sim3Solver.cc
src/Viewer.cc
src/ImuTypes.cc
src/G2oTypes.cc
src/G2oLineTypes.cc
src/CameraModels/Pinhole.cpp
src/CameraModels/KannalaBrandt8.cpp
src/OptimizableTypes.cpp
src/OptimizableLineTypes.cpp
src/MLPnPsolver.cpp
src/GeometricTools.cc
src/TwoViewReconstruction.cc
src/Config.cc
src/Settings.cc
include/System.h
include/Tracking.h
include/LocalMapping.h
include/LoopClosing.h
include/ORBextractor.h
include/ORBmatcher.h
include/FrameDrawer.h
include/Converter.h
include/MapPoint.h
include/KeyFrame.h
include/Atlas.h
include/Map.h
include/MapDrawer.h
include/Optimizer.h
include/Frame.h
include/KeyFrameDatabase.h
include/Sim3Solver.h
include/Viewer.h
include/ImuTypes.h
include/G2oTypes.h
include/G2oLineTypes.h
include/CameraModels/GeometricCamera.h
include/CameraModels/Pinhole.h
include/CameraModels/KannalaBrandt8.h
include/OptimizableTypes.h
include/OptimizableLineTypes.h
include/MLPnPsolver.h
include/GeometricTools.h
include/TwoViewReconstruction.h
include/SerializationUtils.h
include/Config.h
include/Settings.h
###
src/MapLine.cc
src/LineExtractor.cc
src/LineMatcher.cc
src/MapObject.cc
src/Pointers.cc
###
src/PointCloudMapping.cc
src/PointCloudKeyFrame.cc
src/PointCloudDrawer.cc
src/OctomapManager.cc
src/ColorOctomapServer.cc
src/OctreePointCloudCentroid.cc
src/LabelMap.cc
#src/FastFusionManager.cc
src/PointCloudMap.cc
src/PointCloudMapChisel.cc
src/PointCloudMapFastFusion.cc
src/PointCloudMapVoxblox.cc
src/PointCloudMapOctreePointCloud.cc
src/PointCloudMapOctomap.cc
src/PointCloudMapVoxelGridFilter.cc
src/StereoDisparity.cc
src/KeyFrameSearchTree.cc
src/PointCloudAtlas.cc
src/PointCloudMapTypes.cc
### 
src/GlObject.cc
src/GlPointCloud.cc
src/GlObjectList.cc
###
src/g2o/types_sba_line.cpp
src/g2o/types_six_dof_expmap2.cpp
src/g2o/types_seven_dof_expmap2.cpp
)

#add_subdirectory(Thirdparty/g2o)

#target_link_libraries(${PROJECT_NAME}
#${OpenCV_LIBS}
#${EIGEN3_LIBS}
#${Pangolin_LIBRARIES}
#${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
#${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
#-lboost_serialization
#-lcrypto
#)

if (WITH_CUDA)
    cuda_compile(CUDA_OBJS ${CUDA_SOURCES})
endif()

if(WIN32)
    # Abseil's exported targets (pulled in transitively by PCL/protobuf and the
    # vcpkg static glog) carry the bare MSVC lib flag "-ignore:4221" wrapped in a
    # $<LINK_ONLY:...> generator expression in their INTERFACE_LINK_LIBRARIES. The
    # vcpkg toolchain forwards it to the linker, but the all-clang driver used here
    # treats it as an unknown argument and fails the final link. Strip it from
    # every imported Abseil target (it only suppresses a benign "no public symbols"
    # archive warning). Done here, after all find_package calls, so nothing
    # re-imports Abseil and restores the flag. The target names are read from the
    # exported targets file because Abseil imports them GLOBAL (not enumerable via a
    # directory's IMPORTED_TARGETS).
    file(GLOB _absl_target_files "${CMAKE_PREFIX_PATH}/share/absl/abslTargets.cmake")
    set(_absl_names "")
    foreach(_f IN LISTS _absl_target_files)
        file(STRINGS "${_f}" _lines REGEX "add_library\\(absl::")
        foreach(_l IN LISTS _lines)
            string(REGEX MATCH "absl::[A-Za-z0-9_]+" _n "${_l}")
            if(_n)
                list(APPEND _absl_names "${_n}")
            endif()
        endforeach()
    endforeach()
    list(REMOVE_DUPLICATES _absl_names)
    foreach(_t IN LISTS _absl_names)
        if(TARGET ${_t})
            get_target_property(_ill ${_t} INTERFACE_LINK_LIBRARIES)
            if(_ill)
                set(_clean "")
                foreach(_item IN LISTS _ill)
                    if(NOT _item MATCHES "ignore:4221")
                        list(APPEND _clean "${_item}")
                    endif()
                endforeach()
                set_target_properties(${_t} PROPERTIES INTERFACE_LINK_LIBRARIES "${_clean}")
            endif()
        endif()
    endforeach()
endif()

if(USE_HIP)
    add_library(${PROJECT_NAME} ${LIBRARY_TYPE}
        ${HIP_SOURCES}
        ${CPU_SOURCES}
    )
    set_target_properties(${PROJECT_NAME} PROPERTIES HIP_ARCHITECTURES "${CMAKE_HIP_ARCHITECTURES}")
elseif(WITH_CUDA)
    add_library(${PROJECT_NAME} ${LIBRARY_TYPE}
        ${CUDA_OBJS}
        ${CPU_SOURCES}
    )
else()
    add_library(${PROJECT_NAME} ${LIBRARY_TYPE}
        ${CPU_SOURCES}
    )
endif()

target_link_libraries(${PROJECT_NAME}
${EXTERNAL_LIBS} ${EXTERNAL_CORE_LIBS})


set(CORE_LIBS ${PROJECT_NAME})

option(PLVS_BUILD_TESTS "Build focused mapping regression tests" OFF)
if(PLVS_BUILD_TESTS)
    enable_testing()
    add_executable(test_line_mapping tests/test_line_mapping.cc)
    set_target_properties(test_line_mapping PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/tests")
    target_link_libraries(test_line_mapping ${CORE_LIBS} ${EXTERNAL_LIBS} ${EXTERNAL_CORE_LIBS})
    foreach(line_test observations one_sided replacement distance fusion triangulation)
        add_test(NAME line_${line_test} COMMAND test_line_mapping ${line_test})
    endforeach()
    add_executable(test_octree_cleanup tests/test_octree_cleanup.cc)
    set_target_properties(test_octree_cleanup PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/tests")
    target_link_libraries(test_octree_cleanup ${CORE_LIBS} ${EXTERNAL_LIBS} ${EXTERNAL_CORE_LIBS})
    add_test(NAME octree_cleanup_no_segm COMMAND test_octree_cleanup)
    add_test(NAME octree_cleanup_segm COMMAND test_octree_cleanup segm)
endif()

message(STATUS "Boost_LIBRARIES: ${Boost_LIBRARIES}")
message(STATUS "EXTERNAL_CORE_LIBS: ${EXTERNAL_CORE_LIBS}")


### Build examples


# RGB-D examples
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/RGB-D)

add_executable(rgbd_tum
        Examples/RGB-D/rgbd_tum.cc)
target_link_libraries(rgbd_tum ${CORE_LIBS} ${EXTERNAL_LIBS} ${EXTERNAL_CORE_LIBS})

if(realsense2_FOUND)
    add_executable(rgbd_realsense_D435i
            Examples/RGB-D/rgbd_realsense_D435i.cc)
    target_link_libraries(rgbd_realsense_D435i ${CORE_LIBS} ${EXTERNAL_LIBS} ${EXTERNAL_CORE_LIBS})
endif()


# RGB-D inertial examples
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/RGB-D-Inertial)

if(realsense2_FOUND)
    add_executable(rgbd_inertial_realsense_D435i
            Examples/RGB-D-Inertial/rgbd_inertial_realsense_D435i.cc)
    target_link_libraries(rgbd_inertial_realsense_D435i ${CORE_LIBS} ${EXTERNAL_LIBS} ${EXTERNAL_CORE_LIBS})
endif()

# Stereo examples
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Stereo)

add_executable(stereo_kitti
        Examples/Stereo/stereo_kitti.cc)
target_link_libraries(stereo_kitti ${CORE_LIBS} ${EXTERNAL_LIBS} ${EXTERNAL_CORE_LIBS})

add_executable(stereo_euroc
        Examples/Stereo/stereo_euroc.cc)
target_link_libraries(stereo_euroc ${CORE_LIBS} ${EXTERNAL_LIBS} ${EXTERNAL_CORE_LIBS})

add_executable(stereo_tum_vi
        Examples/Stereo/stereo_tum_vi.cc)
target_link_libraries(stereo_tum_vi ${CORE_LIBS} ${EXTERNAL_LIBS} ${EXTERNAL_CORE_LIBS})

if(realsense2_FOUND)
    add_executable(stereo_realsense_t265
            Examples/Stereo/stereo_realsense_t265.cc)
    target_link_libraries(stereo_realsense_t265 ${CORE_LIBS} ${EXTERNAL_LIBS} ${EXTERNAL_CORE_LIBS})

    add_executable(stereo_realsense_D435i
            Examples/Stereo/stereo_realsense_D435i.cc)
    target_link_libraries(stereo_realsense_D435i ${CORE_LIBS} ${EXTERNAL_LIBS} ${EXTERNAL_CORE_LIBS})
endif()

# Monocular examples
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Monocular)

add_executable(mono_tum
        Examples/Monocular/mono_tum.cc)
target_link_libraries(mono_tum ${CORE_LIBS} ${EXTERNAL_LIBS} ${EXTERNAL_CORE_LIBS})

add_executable(mono_kitti
        Examples/Monocular/mono_kitti.cc)
target_link_libraries(mono_kitti ${CORE_LIBS} ${EXTERNAL_LIBS} ${EXTERNAL_CORE_LIBS})

add_executable(mono_euroc
        Examples/Monocular/mono_euroc.cc)
target_link_libraries(mono_euroc ${CORE_LIBS} ${EXTERNAL_LIBS} ${EXTERNAL_CORE_LIBS})

add_executable(mono_tum_vi
        Examples/Monocular/mono_tum_vi.cc)
target_link_libraries(mono_tum_vi ${CORE_LIBS} ${EXTERNAL_LIBS} ${EXTERNAL_CORE_LIBS})

if(realsense2_FOUND)
    add_executable(mono_realsense_t265
            Examples/Monocular/mono_realsense_t265.cc)
    target_link_libraries(mono_realsense_t265 ${CORE_LIBS} ${EXTERNAL_LIBS} ${EXTERNAL_CORE_LIBS})

    add_executable(mono_realsense_D435i
            Examples/Monocular/mono_realsense_D435i.cc)
    target_link_libraries(mono_realsense_D435i ${CORE_LIBS} ${EXTERNAL_LIBS} ${EXTERNAL_CORE_LIBS})
endif()

# Monocular inertial examples
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Monocular-Inertial)

add_executable(mono_inertial_euroc
        Examples/Monocular-Inertial/mono_inertial_euroc.cc)
target_link_libraries(mono_inertial_euroc ${CORE_LIBS} ${EXTERNAL_LIBS} ${EXTERNAL_CORE_LIBS})

add_executable(mono_inertial_tum_vi
        Examples/Monocular-Inertial/mono_inertial_tum_vi.cc)
target_link_libraries(mono_inertial_tum_vi ${CORE_LIBS} ${EXTERNAL_LIBS} ${EXTERNAL_CORE_LIBS})

if(realsense2_FOUND)
    add_executable(mono_inertial_realsense_t265
            Examples/Monocular-Inertial/mono_inertial_realsense_t265.cc)
    target_link_libraries(mono_inertial_realsense_t265 ${CORE_LIBS} ${EXTERNAL_LIBS} ${EXTERNAL_CORE_LIBS})

    add_executable(mono_inertial_realsense_D435i
            Examples/Monocular-Inertial/mono_inertial_realsense_D435i.cc)
    target_link_libraries(mono_inertial_realsense_D435i ${CORE_LIBS} ${EXTERNAL_LIBS} ${EXTERNAL_CORE_LIBS})
endif()

# Stereo Inertial examples
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Stereo-Inertial)

add_executable(stereo_inertial_euroc
        Examples/Stereo-Inertial/stereo_inertial_euroc.cc)
target_link_libraries(stereo_inertial_euroc ${CORE_LIBS} ${EXTERNAL_LIBS} ${EXTERNAL_CORE_LIBS})

add_executable(stereo_inertial_tum_vi
        Examples/Stereo-Inertial/stereo_inertial_tum_vi.cc)
target_link_libraries(stereo_inertial_tum_vi ${CORE_LIBS} ${EXTERNAL_LIBS} ${EXTERNAL_CORE_LIBS})

if(realsense2_FOUND)
    add_executable(stereo_inertial_realsense_t265
            Examples/Stereo-Inertial/stereo_inertial_realsense_t265.cc)
    target_link_libraries(stereo_inertial_realsense_t265 ${CORE_LIBS} ${EXTERNAL_LIBS} ${EXTERNAL_CORE_LIBS})

    add_executable(stereo_inertial_realsense_D435i
            Examples/Stereo-Inertial/stereo_inertial_realsense_D435i.cc)
    target_link_libraries(stereo_inertial_realsense_D435i ${CORE_LIBS} ${EXTERNAL_LIBS} ${EXTERNAL_CORE_LIBS})
endif()

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Calibration)
if(realsense2_FOUND)
    add_executable(recorder_realsense_D435i
            Examples/Calibration/recorder_realsense_D435i.cc)
    target_link_libraries(recorder_realsense_D435i ${CORE_LIBS} ${EXTERNAL_LIBS} ${EXTERNAL_CORE_LIBS})

    add_executable(recorder_realsense_T265
            Examples/Calibration/recorder_realsense_T265.cc)
    target_link_libraries(recorder_realsense_T265 ${CORE_LIBS} ${EXTERNAL_LIBS} ${EXTERNAL_CORE_LIBS})
endif()


###  Old examples

# RGB-D examples
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples_old/RGB-D)

add_executable(rgbd_tum_old
        Examples_old/RGB-D/rgbd_tum.cc)
target_link_libraries(rgbd_tum_old ${CORE_LIBS} ${EXTERNAL_LIBS} ${EXTERNAL_CORE_LIBS})


add_executable(rgbd_tum_ar 
        Examples_old/RGB-D/rgbd_tum_ar.cc Examples_old/RGB-D/ViewerAR.h Examples_old/RGB-D/ViewerAR.cc)
target_link_libraries(rgbd_tum_ar ${CORE_LIBS} ${EXTERNAL_LIBS} ${EXTERNAL_CORE_LIBS})

add_executable(rgbd_inseg
	Examples_old/RGB-D/rgbd_inseg.cc)
target_link_libraries(rgbd_inseg ${CORE_LIBS} ${EXTERNAL_LIBS} ${EXTERNAL_CORE_LIBS})

if(realsense2_FOUND)
    add_executable(rgbd_realsense_D435i_old
            Examples_old/RGB-D/rgbd_realsense_D435i.cc)
    target_link_libraries(rgbd_realsense_D435i_old ${CORE_LIBS} ${EXTERNAL_LIBS} ${EXTERNAL_CORE_LIBS})
endif()


# RGB-D inertial examples
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples_old/RGB-D-Inertial)

if(realsense2_FOUND)
    add_executable(rgbd_inertial_realsense_D435i_old
            Examples_old/RGB-D-Inertial/rgbd_inertial_realsense_D435i.cc)
    target_link_libraries(rgbd_inertial_realsense_D435i_old ${CORE_LIBS} ${EXTERNAL_LIBS} ${EXTERNAL_CORE_LIBS})
endif()

# Stereo examples
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples_old/Stereo)

add_executable(stereo_kitti_old
        Examples_old/Stereo/stereo_kitti.cc)
target_link_libraries(stereo_kitti_old ${CORE_LIBS} ${EXTERNAL_LIBS} ${EXTERNAL_CORE_LIBS})

add_executable(stereo_euroc_old
        Examples_old/Stereo/stereo_euroc.cc)
target_link_libraries(stereo_euroc_old ${CORE_LIBS} ${EXTERNAL_LIBS} ${EXTERNAL_CORE_LIBS})

add_executable(stereo_tum_vi_old
        Examples_old/Stereo/stereo_tum_vi.cc)
target_link_libraries(stereo_tum_vi_old ${CORE_LIBS} ${EXTERNAL_LIBS} ${EXTERNAL_CORE_LIBS})

if(realsense2_FOUND)
    add_executable(stereo_realsense_t265_old
            Examples_old/Stereo/stereo_realsense_t265.cc)
    target_link_libraries(stereo_realsense_t265_old ${CORE_LIBS} ${EXTERNAL_LIBS} ${EXTERNAL_CORE_LIBS})

    add_executable(stereo_realsense_D435i_old
            Examples_old/Stereo/stereo_realsense_D435i.cc)
    target_link_libraries(stereo_realsense_D435i_old ${CORE_LIBS} ${EXTERNAL_LIBS} ${EXTERNAL_CORE_LIBS})
endif()

# Monocular examples
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples_old/Monocular)

add_executable(mono_tum_old
        Examples_old/Monocular/mono_tum.cc)
target_link_libraries(mono_tum_old ${CORE_LIBS} ${EXTERNAL_LIBS} ${EXTERNAL_CORE_LIBS})

add_executable(mono_kitti_old
        Examples_old/Monocular/mono_kitti.cc)
target_link_libraries(mono_kitti_old ${CORE_LIBS} ${EXTERNAL_LIBS} ${EXTERNAL_CORE_LIBS})

add_executable(mono_euroc_old
        Examples_old/Monocular/mono_euroc.cc)
target_link_libraries(mono_euroc_old ${CORE_LIBS} ${EXTERNAL_LIBS} ${EXTERNAL_CORE_LIBS})

add_executable(mono_tum_vi_old
        Examples_old/Monocular/mono_tum_vi.cc)
target_link_libraries(mono_tum_vi_old ${CORE_LIBS} ${EXTERNAL_LIBS} ${EXTERNAL_CORE_LIBS})

if(realsense2_FOUND)
    add_executable(mono_realsense_t265_old
            Examples_old/Monocular/mono_realsense_t265.cc)
    target_link_libraries(mono_realsense_t265_old ${CORE_LIBS} ${EXTERNAL_LIBS} ${EXTERNAL_CORE_LIBS})

    add_executable(mono_realsense_D435i_old
            Examples_old/Monocular/mono_realsense_D435i.cc)
    target_link_libraries(mono_realsense_D435i_old ${CORE_LIBS} ${EXTERNAL_LIBS} ${EXTERNAL_CORE_LIBS})
endif()

# Monocular inertial examples
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples_old/Monocular-Inertial)

add_executable(mono_inertial_euroc_old
        Examples_old/Monocular-Inertial/mono_inertial_euroc.cc)
target_link_libraries(mono_inertial_euroc_old ${CORE_LIBS} ${EXTERNAL_LIBS} ${EXTERNAL_CORE_LIBS})

add_executable(mono_inertial_tum_vi_old
        Examples_old/Monocular-Inertial/mono_inertial_tum_vi.cc)
target_link_libraries(mono_inertial_tum_vi_old ${CORE_LIBS} ${EXTERNAL_LIBS} ${EXTERNAL_CORE_LIBS})

if(realsense2_FOUND)
    add_executable(mono_inertial_realsense_t265_old
            Examples_old/Monocular-Inertial/mono_inertial_realsense_t265.cc)
    target_link_libraries(mono_inertial_realsense_t265_old ${CORE_LIBS} ${EXTERNAL_LIBS} ${EXTERNAL_CORE_LIBS})

    add_executable(mono_inertial_realsense_D435i_old
            Examples_old/Monocular-Inertial/mono_inertial_realsense_D435i.cc)
    target_link_libraries(mono_inertial_realsense_D435i_old ${CORE_LIBS} ${EXTERNAL_LIBS} ${EXTERNAL_CORE_LIBS})
endif()

# Stereo Inertial examples
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples_old/Stereo-Inertial)

add_executable(stereo_inertial_euroc_old
        Examples_old/Stereo-Inertial/stereo_inertial_euroc.cc)
target_link_libraries(stereo_inertial_euroc_old ${CORE_LIBS} ${EXTERNAL_LIBS} ${EXTERNAL_CORE_LIBS})

add_executable(stereo_inertial_tum_vi_old
        Examples_old/Stereo-Inertial/stereo_inertial_tum_vi.cc)
target_link_libraries(stereo_inertial_tum_vi_old ${CORE_LIBS} ${EXTERNAL_LIBS} ${EXTERNAL_CORE_LIBS})

if(realsense2_FOUND)
    add_executable(stereo_inertial_realsense_t265_old
            Examples_old/Stereo-Inertial/stereo_inertial_realsense_t265.cc)
    target_link_libraries(stereo_inertial_realsense_t265_old ${CORE_LIBS} ${EXTERNAL_LIBS} ${EXTERNAL_CORE_LIBS})

    add_executable(stereo_inertial_realsense_D435i_old
            Examples_old/Stereo-Inertial/stereo_inertial_realsense_D435i.cc)
    target_link_libraries(stereo_inertial_realsense_D435i_old ${CORE_LIBS} ${EXTERNAL_LIBS} ${EXTERNAL_CORE_LIBS})
endif()


set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Vocabulary)
add_executable(bin_vocabulary Vocabulary/bin_vocabulary.cpp)
target_link_libraries(bin_vocabulary ${CORE_LIBS} ${EXTERNAL_LIBS} ${EXTERNAL_CORE_LIBS})

if(EXISTS ${PROJECT_SOURCE_DIR}/test)
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
    #add_subdirectory(${PROJECT_SOURCE_DIR}/test) # uncomment to build tests/examples
endif()

message(STATUS "CMAKE_CXX_FLAGS: ${CMAKE_CXX_FLAGS}")
message(STATUS "CUDA_NVCC_FLAGS: ${CUDA_NVCC_FLAGS}")

