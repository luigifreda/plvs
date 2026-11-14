cmake_minimum_required(VERSION 3.1)

set(CPP_STANDARD_VERSION "17" CACHE STRING "Desired C++ standard version")

include_directories(../include)

file(GLOB STEREOSRCS "*.cu" "*.cpp")

if(USE_HIP)
    # HIP build
    set_source_files_properties(${STEREOSRCS} PROPERTIES LANGUAGE HIP)
    if(WIN32)
      set(CMAKE_HIP_FLAGS "${CMAKE_HIP_FLAGS} -std=c++${CMAKE_CXX_STANDARD} -O3")
    else()
      set(CMAKE_HIP_FLAGS "${CMAKE_HIP_FLAGS} -std=c++${CMAKE_CXX_STANDARD} -O3 -fPIC")
    endif()

    if(LIBSGM_SHARED)
        add_library(sgm SHARED stereo_sgm.cpp ${STEREOSRCS})
    else()
        add_library(sgm STATIC stereo_sgm.cpp ${STEREOSRCS})
    endif()
    set_target_properties(sgm PROPERTIES HIP_ARCHITECTURES "${CMAKE_HIP_ARCHITECTURES}")
else()
    # CUDA build
    find_package(CUDA REQUIRED)

    if (CMAKE_COMPILER_IS_GNUCXX)
        set(CMAKE_CXX_FLAGS "-O3 -Wall -march=native -fPIC -DNDEBUG")
        set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -std=c++${CMAKE_CXX_STANDARD}")
    endif()

    SET(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} ${CUDA_ARCH}")

    if(LIBSGM_SHARED)
        CUDA_ADD_LIBRARY(sgm stereo_sgm.cpp ${STEREOSRCS} SHARED)
        target_link_libraries(sgm ${CUDA_LIBRARIES})
    else()
        CUDA_ADD_LIBRARY(sgm stereo_sgm.cpp ${STEREOSRCS} STATIC)
    endif()
endif()

install(
	TARGETS sgm
	ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
	LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
	RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
)

install(
	DIRECTORY ${CMAKE_SOURCE_DIR}/include
	DESTINATION ${CMAKE_INSTALL_PREFIX}
	FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

install(
	FILES ${CMAKE_SOURCE_DIR}/FindLibSGM.cmake
	DESTINATION ${CMAKE_INSTALL_PREFIX}
)

## install lib

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
