cmake_minimum_required(VERSION 3.1)

set(CPP_STANDARD_VERSION "17" CACHE STRING "Desired C++ standard version") # this is the first target version that is searched for 

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../movie/cmake)

message(STATUS ${CMAKE_MODULE_PATH})

find_package(CUDA REQUIRED)

find_package(OpenCV REQUIRED core) # dummy
if(OpenCV_VERSION VERSION_LESS 3.0) 
	find_package(OpenCV REQUIRED core highgui imgproc)  # 2.x
else()
	find_package(OpenCV REQUIRED core highgui imgproc imgcodecs videoio) # 3.x
endif()

find_package(OpenGL REQUIRED)

find_package(GLEW REQUIRED)

find_package(GLFW REQUIRED)

include_directories(../../include)
include_directories(${CUDA_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${GLEW_INCLUDE_DIRS})
include_directories(${GLFW_INCLUDE_DIRS})

include_directories(${ZED_SDK_INCLUDE_DIR})

if(NOT WIN32)
	set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -std=c++17")
endif()


CUDA_ADD_EXECUTABLE(zed_demo
	zed_demo.cpp
	demo.cpp
	renderer.cpp
	cuda_gl.cu
)


TARGET_LINK_LIBRARIES(zed_demo 
	sgm 
	${CUDA_LIBRARIES} ${CUDA_nppi_LIBRARY} 
	${OpenCV_LIBS} 
	${GLEW_LIBRARIES} ${GLFW_LIBRARY} ${OPENGL_LIBRARY}
	${ZED_SDK_LIB}
)
