cmake_minimum_required(VERSION 3.1)

set(CPP_STANDARD_VERSION "17" CACHE STRING "Desired C++ standard version") # this is the first target version that is searched for 

find_package(CUDA REQUIRED)

include_directories(../include)

if (CMAKE_COMPILER_IS_GNUCXX)
	set(CMAKE_CXX_FLAGS "-O3 -Wall -march=native -fPIC -DNDEBUG")
	set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -std=c++${CMAKE_CXX_STANDARD}")
endif()

SET(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} ${CUDA_ARCH}")

file(GLOB STEREOSRCS "*.cu" "*.cpp")

if(LIBSGM_SHARED)
	CUDA_ADD_LIBRARY(sgm stereo_sgm.cpp ${STEREOSRCS} SHARED)
	target_link_libraries(sgm ${CUDA_LIBRARIES})
else()
	CUDA_ADD_LIBRARY(sgm stereo_sgm.cpp ${STEREOSRCS} STATIC)
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
